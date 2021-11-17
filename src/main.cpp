#include "json.h"
#include "route.h"

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <fstream>
#include <memory>
#include <iomanip>

using namespace std;



struct Coordinates {
    double lat; //latitude
    double lon; //longitude
};


using DistanceMap = unordered_map<string, double>; 


struct StopParams {
    Coordinates coords;
    DistanceMap distance;
};


struct BusRoute {
    vector<string> stops;
    bool isCyclic;
};


struct BusStats {
    double routeLength;
    double routeLengthNew;
    double routeCoef;

    int totalStops;
    int uniqueStops;
};


struct StopStats {
    set<string> buses;
};


enum class RequestType {
    Bus,
    Stop,
    Route
};


struct Request {
    RequestType type;
    string name;
    int  requestId;
    string name2; 
};


struct EdgeAction {
    string type;
    double time;
    string name;
    unsigned int spans;
};

struct RouteAction {
    bool notFound;
    double totalTime;
    vector<EdgeAction> actions;
};


class Requests {

public:
    Requests() = default; 

    void readRequestsJson(Json::Document& document) {
        auto mainNode = document.GetRoot().AsMap();
        auto baseRequests = mainNode["base_requests"];
        auto statRequests = mainNode["stat_requests"];
        readInputRequestsJson(baseRequests);
        readOutputRequestsJson(statRequests);

        if (mainNode.count("routing_settings")) {
            auto routeSettings = mainNode["routing_settings"].AsMap();
            busWaitTime = routeSettings["bus_wait_time"].AsInt();
            busVelocity = routeSettings["bus_velocity"].AsInt();
        }

        build(); //Построение базовых элементов
        createGraphAndRoute(); //Построение графа
    }


    RouteAction findRoute(const string& from, const string& to) {
        size_t fromIdx = 2 * stopsIdx[from];
        size_t toIdx = 2 * stopsIdx[to];
        auto foundRoute = router->BuildRoute(fromIdx, toIdx);

        RouteAction routeAction;
        if (foundRoute.has_value() == false) {
            routeAction.notFound = true;
            return routeAction;
        }
        routeAction.notFound = false;

        vector<EdgeAction> routeActions;
        auto id = foundRoute.value().id;

        for (size_t i = 0; i < foundRoute.value().edge_count; ++i) {
            auto edgeIdx = router->GetRouteEdge(id, i);
            routeAction.actions.push_back(edgeActions[edgeIdx]);
        }

        routeAction.totalTime = foundRoute.value().weight;
        return routeAction;
    }


private:


    void createGraphAndRoute() {
        graph = make_unique<Graph::DirectedWeightedGraph<double>>(2 * stops.size());
        for (size_t i = 0; i < stopsNames.size(); ++i) {
            graph->AddEdge(Graph::Edge<double>{ 2 * i, 2 * i + 1, static_cast<double>(busWaitTime)});
            edgeActions.push_back(EdgeAction{"Wait", busWaitTime, stopsNames[i], 0 });
        } 
        for (const auto& [busName, bus] : routes) {
            const auto& busStops = bus.stops;

            for (size_t i = 0; i < busStops.size(); ++i) {
                double totalTime{};
                unsigned int spanCount{};
                for (size_t j = i + 1; j < busStops.size(); ++j) {
                    size_t idx1 = stopsIdx[busStops[j - 1]];
                    size_t idx2 = stopsIdx[busStops[j]];
                    totalTime += stopsDist[idx1][idx2] / (busVelocity * 1000 / 60);
                    graph->AddEdge(Graph::Edge<double>{ 2 * stopsIdx[busStops[i]] + 1, 2 * idx2, totalTime });
                    edgeActions.push_back(EdgeAction{ "Bus", totalTime, busName, ++spanCount });
                }
            }

            if (bus.isCyclic == false) 
                for (size_t i = 0; i < busStops.size(); ++i) { 
                    double totalTime{};
                    unsigned int spanCount{};
                    for (int j = i - 1; j >= 0; --j) {
                        size_t idx1 = stopsIdx[busStops[j]];   
                        size_t idx2 = stopsIdx[busStops[j + 1]];
                        totalTime += stopsDist[idx2][idx1] / (busVelocity * 1000 / 60); 
                        graph->AddEdge(Graph::Edge<double>{ 2 * stopsIdx[busStops[i]] + 1, 2 * idx1, totalTime });
                        edgeActions.push_back(EdgeAction{ "Bus", totalTime, busName, ++spanCount });
                    }
                }

        }
        router = make_unique<Graph::Router<double>>(*graph);
    }



    void build() {

        for (auto& r : routes) {
            BusStats stats;

            const auto& busName = r.first;

            stats.routeLength = calculateLength(r.second.stops, r.second.isCyclic);
            stats.routeLengthNew = calculateLengthNew(r.second.stops, r.second.isCyclic);

            stats.routeCoef = stats.routeLengthNew / stats.routeLength;

            stats.totalStops = r.second.isCyclic ? r.second.stops.size() : r.second.stops.size() * 2 - 1;

            unordered_set<string> uniqueStops(r.second.stops.begin(), r.second.stops.end());
            stats.uniqueStops = uniqueStops.size();

            for (const auto& s : uniqueStops)
                stopStats[s].buses.insert(busName); 

            busStats[busName] = stats;
        }
    }

  

private:

    unordered_map<string, StopParams> stops; 
    unordered_map<string, BusRoute> routes;

    vector<Request> requests;

    unordered_map<string, BusStats> busStats;
    unordered_map<string, StopStats> stopStats;

    double busWaitTime; 
    double busVelocity; 

    unique_ptr<Graph::DirectedWeightedGraph<double>> graph{ nullptr };
    unique_ptr<Graph::Router<double>> router{ nullptr };

    vector<string> stopsNames;
    unordered_map<string, size_t> stopsIdx;
    unordered_map<size_t, unordered_map<size_t, unsigned int>> stopsDist;

    vector<EdgeAction> edgeActions;


    void readInputRequestsJson(Json::Node& node) {
        auto allRequests = node.AsArray();
        for (const auto& r : allRequests) {
            auto request = r.AsMap();
            string type = request["type"].AsString();
            
            if (type == "Stop")
                readStopJson(request);
            if (type == "Bus")
                readBusJson(request);
        }
    }

    void readStopJson(map<string, Json::Node>& request) {
        string name = request["name"].AsString();
        double lon = request["longitude"].AsDouble();
        double lat = request["latitude"].AsDouble();

        map<string, Json::Node> distances;
        if (request.count("road_distances"))
            distances = request["road_distances"].AsMap();

        stopStats[name] = {}; 
        size_t idx = stopsNames.size();
        stopsNames.push_back(name);
        stopsIdx[name] = idx; 

        if (distances.empty())
            stops[name] = { { lat, lon } };
        else {
            auto stopsDist = parseStopsDistanceJson(distances);
            stops[name] = { { lat, lon },  stopsDist };
        }
    }


    DistanceMap parseStopsDistanceJson(map<string, Json::Node>& distances) {
        DistanceMap distMap;
        for (auto& d : distances)
            distMap[d.first] = d.second.AsDouble();
        return distMap;
    }


    void readBusJson(map<string, Json::Node>& request) {
        string name = request["name"].AsString();

        BusRoute route;
        route.isCyclic = request["is_roundtrip"].AsBool();

        auto stops = request["stops"].AsArray();
        for (auto s : stops)
            route.stops.push_back(s.AsString());

       routes[name] = route;
    }


    void readOutputRequestsJson(Json::Node& node) {
        auto allRequests = node.AsArray();
        for (const auto& r : allRequests) {
            auto request = r.AsMap();
            string type = request["type"].AsString();
            int id = request["id"].AsInt();

            if (type == "Route") {
                string from = request["from"].AsString();
                string to = request["to"].AsString();
                requests.push_back({ RequestType::Route, from, id, to });
            }
            else {
                string name = request["name"].AsString();
                if (type == "Bus")
                    requests.push_back({ RequestType::Bus, name, id }); 
                if (type == "Stop")
                    requests.push_back({ RequestType::Stop, name, id }); 
            }
        }
    }


public:


    void runJson(ostream& os) {
        os << setprecision(9);

        vector<Json::Node> jsonArray;
        for (const auto& r : requests) { 
            map<string, Json::Node> outputRecord;
            outputRecord["request_id"] = r.requestId;

            if (r.type == RequestType::Bus) {
                if (busStats.count(r.name)) {
                    outputRecord["stop_count"] = busStats[r.name].totalStops;
                    outputRecord["unique_stop_count"] = busStats[r.name].uniqueStops;

                    double routeLendth = busStats[r.name].routeLengthNew;
                    if (routeLendth == round(routeLendth)) {
                        int intLen = routeLendth;
                        outputRecord["route_length"] = intLen;
                    }
                    else
                        outputRecord["route_length"] = routeLendth;

                    outputRecord["curvature"] = busStats[r.name].routeCoef;
                }
                else
                    outputRecord["error_message"] = "not found"s;
            }

            if (r.type == RequestType::Stop) {
                if (stopStats.count(r.name)) {
                    vector<Json::Node> buses;
                    if (stopStats[r.name].buses.empty() == false) 
                        for (const auto& busName : stopStats[r.name].buses)
                            buses.push_back(busName);
                    outputRecord["buses"] = buses;
                }
                else
                    outputRecord["error_message"] = "not found"s;
            }

            if (r.type == RequestType::Route) {
                RouteAction route = findRoute(r.name, r.name2);

                if (route.notFound)
                    outputRecord["error_message"] = "not found"s;
                else {

                    outputRecord["total_time"] = route.totalTime;

                    vector<Json::Node> actions;
                    for (auto& a : route.actions) {
                        map<string, Json::Node> action;
                        action["type"] = a.type;

                        if (a.time == round(a.time)) {
                            int intTime = a.time;
                            action["time"] = intTime;
                        }
                        else
                            action["time"] = a.time;

                        if (a.type == "Wait") 
                            action["stop_name"] = a.name;
                        if (a.type == "Bus") {
                            action["bus"] = a.name;
                            action["span_count"] = int(a.spans);
                        }
                        actions.push_back(action);
                    }
                    outputRecord["items"] = actions;
                }
            }

            jsonArray.push_back(outputRecord);
        }

        os << jsonArray;
    }




private: 


    double calculateLengthNew(const vector<string>& stopNames, bool isCyclic) {

        double totalDistance = 0.;

        for (size_t i = 1; i < stopNames.size(); ++i) {
            const auto& n1 = stopNames[i - 1];
            const auto& n2 = stopNames[i];
            size_t idx1 = stopsIdx[n1];
            size_t idx2 = stopsIdx[n2];

            double dist{};
            if (stops[n1].distance.count(n2)) 
                dist = stops[n1].distance[n2];
            else
                dist = stops[n2].distance[n1];

            stopsDist[idx1][idx2] = dist; 

            totalDistance += dist;
        }

        if (isCyclic == false) { 
            for (size_t i = stopNames.size() - 1; i >= 1; --i) {
                const auto& n1 = stopNames[i];
                const auto& n2 = stopNames[i - 1];
                size_t idx1 = stopsIdx[n1];
                size_t idx2 = stopsIdx[n2];

                double dist{};
                if (stops[n1].distance.count(n2)) 
                    dist = stops[n1].distance[n2];
                else
                    dist = stops[n2].distance[n1];

                stopsDist[idx1][idx2] = dist;

                totalDistance += dist;
            }
        }

        return totalDistance; 
    }


    double calculateLength(const vector<string>& stopNames, bool isCyclic) {
        double totalDistance = 0.;
        for (size_t i = 1; i < stopNames.size(); ++i) {
            const auto& n1 = stopNames[i - 1];
            const auto& n2 = stopNames[i];
            double dist = distanceBetween(stops[n1].coords, stops[n2].coords);
            totalDistance += dist;
        }
        if (isCyclic == false)
            totalDistance *= 2.0; //Round-trip
        return totalDistance * 1000.0 ; //Приводим к метрам
    }


    double distanceBetween(const Coordinates& lhs, const Coordinates& rhs) { 
        static const double p = 3.1415926535 / 180.0;
        static const double EARTH_RADIUS_2 = 6371.0 * 2.0; 
        double a = 0.5 - cos((rhs.lat - lhs.lat) * p) / 2 + cos(lhs.lat * p) 
                    * cos(rhs.lat * p) * (1 - cos((rhs.lon - lhs.lon) * p)) / 2;
        return EARTH_RADIUS_2 * asin(sqrt(a));
    }

};



int main() {

    Requests requests;

    auto json = Json::Load(cin); 
    requests.readRequestsJson(json);
    requests.runJson(cout); 

    return 0;
}
