#include "Planning.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals; 

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {
        // Prevzeti mapy(/map_server/map) z GetMap
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");
        
        // Prevzeti cilovych souradnic pro vypocet trasy (z uzlu plan_path) a vlozeni do funkce planPath
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
            "plan_path", 
            std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // Nastaveni posilani (automaticky -->  i kdyz se rviz zapne pozde posli posledni zpravu )
        auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        // Vytvoreni vysilace (cokoliv posleme na path_pub_ se objevi na topicu planned_path)
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", path_qos);

        RCLCPP_INFO(get_logger(), "Planning node started.");

        // Cekani na mapu 
        while (!map_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the map service.");
                return;
            }
            RCLCPP_INFO(get_logger(), "Waiting for map service to appear...");
        }

        // Vytvoreni prazdne obalky pro mapu
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
        // Asynchronni zadost o mapu (neco jako MCU, pote se zavola funkce mapCallback)
        map_client_->async_send_request(request, std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
    }

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    // Ceka se na mapu, ktera az dojde (future = mapa)
    
    // Ulozeni si odpovedi z future do response (ulozeni si prijate mapy)
    auto response = future.get();

    // Kontrola, jestli je prijata zprava (mapa) je v poradku (neni prazdna, pri ceste nenastala nejaka chyba)
    if (response) {
        map_ = response->map;   // Ulozeni si mapy
        RCLCPP_INFO(get_logger(), "Map successfully received!");
        RCLCPP_INFO(get_logger(), "Resolution: %.2f m/cell", map_.info.resolution);
        RCLCPP_INFO(get_logger(), "Size: %u x %u cells", map_.info.width, map_.info.height);

        dilateMap();    // Zavolani funkce dilateMap
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to fetch the map!");
    }

    // ********
    // * Help *
    // ********
    /*
    auto response = future.get();
    if (response) {
        ...
    }
    */
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    // Prijima data od uzivatele ve formatu request (od kud chce), response (kam chce jet), samotna funkce pote vola celou matiku za planovanim trasy

    RCLCPP_INFO(get_logger(), "=================================");
    RCLCPP_INFO(get_logger(), "Service /plan_path was CALLED!");
    RCLCPP_INFO(get_logger(), "Start position: [x: %.2f, y: %.2f]", request->start.pose.position.x, request->start.pose.position.y);
    RCLCPP_INFO(get_logger(), "Goal position:  [x: %.2f, y: %.2f]", request->goal.pose.position.x, request->goal.pose.position.y);
    RCLCPP_INFO(get_logger(), "=================================");

    // Zavolani vypocetni funkce aStar se startovni pozici a cilovou pozici
    aStar(request->start, request->goal);
    // Zavolani funkce smoothPath pro vyhladeni cesty
    smoothPath();

    // Timestamp, casove razitko pro rviz
    path_.header.stamp = this->get_clock()->now();
    // Nastaveni ke kteremu framu se data vztahuji (ted se vztahuji k map - pocatek map + rozmery map) 
    path_.header.frame_id = "map"; 

    // Publikovani do topicu path_ ktery pote vyuziva rviz
    path_pub_->publish(path_);
    // Odeslani cesty zadateli
    response->plan = path_;

    // ********
    // * Help *
    // ********
    /*
    aStar(request->start, request->goal);
    smoothPath();

    path_pub_->publish(path_);
    */
}

void PlanningNode::dilateMap() {
    // Bezpecnostni zkouska jestli vubec mame mapu, aby jsme nesahali do prazdne pameti
    if (map_.data.empty()) return;

    // Vytvoreni si kopie mapy pro nafukovani (aby jsme si neupravovali original)
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    int radius = 7; // Nafouknuti o 4 bunky (cca 20cm)
    int width = map_.info.width;
    int height = map_.info.height;

    // Pruchod celou mapou
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;  // mapa je 1D pole, takze takhle si vezmu kazdy bod z X,Y souradnic a dam ho do index
            if (map_.data[index] > 50) {    // Pokud je na indexu prekazka 50+ zacni nafukovat
                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dx = -radius; dx <= radius; ++dx) {    //  Kresleni 9x9 pole okolo indexu
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {  //  Kontrola jestli neni dany bod mimo mapu (na kraji)
                            if (dx*dx + dy*dy <= radius*radius) {
                                dilatedMap.data[ny * width + nx] = 100; // Zvetseni prekazky na nasi nafouknute mape
                            }
                        }
                    }
                }
            }
        }
    }
    map_ = dilatedMap;  // Poslani samotne nafoukle mapy do obehu
    RCLCPP_INFO(get_logger(), "Map dilated successfully.");

    // ********
    // * Help *
    // ********
    /*
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    ... processing ...
    map_ = dilatedMap;
    */
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    path_.poses.clear();
    path_.header.frame_id = "map";

    if (map_.data.empty()) return;

    // Prevod metru do souradnic
    int start_x = (start.pose.position.x - map_.info.origin.position.x) / map_.info.resolution;
    int start_y = (start.pose.position.y - map_.info.origin.position.y) / map_.info.resolution;
    int goal_x = (goal.pose.position.x - map_.info.origin.position.x) / map_.info.resolution;
    int goal_y = (goal.pose.position.y - map_.info.origin.position.y) / map_.info.resolution;

    // Informace o mape
    int width = map_.info.width;
    int height = map_.info.height;

    // Kontrola, ze souradnice startu a cile se nachazeji na mape
    if (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height ||
        goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height) {
        RCLCPP_ERROR(get_logger(), "Start or goal is out of bounds!");
        return;
    }

    std::vector<bool> closedList(width * height, false);    // Closedlist-tam kde jsme uz byli (aby jsme nechodili v kruzich)
    std::vector<std::shared_ptr<Cell>> cellGrid(width * height, nullptr); // Pamet pro cestu
    std::vector<std::shared_ptr<Cell>> openList;    // Fronta prace (objekty se kterymi pracujeme)

    // Inicializace promennych f,h,g
    auto startNode = std::make_shared<Cell>(start_x, start_y);
    startNode->g = 0.0;
    startNode->h = std::hypot(goal_x - start_x, goal_y - start_y);
    startNode->f = startNode->g + startNode->h;
    
    // Inicializace startovniho bodu a sousedu
    cellGrid[start_y * width + start_x] = startNode;
    openList.push_back(startNode);

    // Inicializace smeru
    std::vector<std::pair<int, int>> directions = {
        {1,0}, {0,1}, {-1,0}, {0,-1}, {1,1}, {-1,1}, {1,-1}, {-1,-1}
    };

    // Inicializace ciloveho pole
    std::shared_ptr<Cell> finalNode = nullptr;

    // Hlavni smycka pro prohledavani 
    while(!openList.empty() && rclcpp::ok()) {  //Prohledavame dokud mame kam jit, jakmile projdeme vse skonci
        auto current_it = openList.begin();
        for (auto it = openList.begin(); it != openList.end(); ++it) {  // Hlednani nejslibnejsiho uzli podle f
            if ((*it)->f < (*current_it)->f) current_it = it;
        }

        // Posuneme se o uzel vpred (ten nejslibnejsi se stane aktualnim)
        auto current = *current_it;
        openList.erase(current_it);

        // Oznaceni uzlu kde jsme byli za projity
        int curr_idx = current->y * width + current->x;
        if (closedList[curr_idx]) continue;
        closedList[curr_idx] = true;

        // Nalezeni cile --> vylez ze smicky
        if (current->x == goal_x && current->y == goal_y) {
            finalNode = current;
            break;
        }

        // Pruzkum vsech ostatnich souseru (8)
        for (const auto& dir : directions) {
            int nx = current->x + dir.first;
            int ny = current->y + dir.second;

            // Ignorace sousedu lezicich mimo mapu
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

            // Vyrazeni sousedu pokud jsou : byl jsem tu, zed, neznam hodnotu prezejzu souseda
            int n_idx = ny * width + nx;
            if (closedList[n_idx] || map_.data[n_idx] > 50 || map_.data[n_idx] == -1) continue;

            // Vypocet cesty
            float move_cost = (dir.first != 0 && dir.second != 0) ? 1.414f : 1.0f;
            float tentative_g = current->g + move_cost;

            // Podivame se do pameti, jestli uz tento bod nezname (nespocitali jsme pro nej heuristiku)
            auto neighbor = cellGrid[n_idx];
            if (!neighbor) {    // Pokud jej nezname, spocitame heuristiku
                neighbor = std::make_shared<Cell>(nx, ny);
                neighbor->h = std::hypot(goal_x - nx, goal_y - ny);
                cellGrid[n_idx] = neighbor;
            }

            // Aktualizace cesty - podle hodnoty souseda a ulozeni si cesty do rodice
            if (tentative_g < neighbor->g || neighbor->g == 0.0) {
                neighbor->parent = current;
                neighbor->g = tentative_g;
                neighbor->f = neighbor->g + neighbor->h;
                openList.push_back(neighbor);
            }
        }
    }

    // Oznameni vysledku
    if (finalNode != nullptr) {
        auto curr = finalNode;
        while (curr != nullptr) {
            geometry_msgs::msg::PoseStamped p;
            p.header.stamp = this->get_clock()->now();
            p.header.frame_id = "map";
            p.pose.position.x = curr->x * map_.info.resolution + map_.info.origin.position.x + (map_.info.resolution / 2.0);
            p.pose.position.y = curr->y * map_.info.resolution + map_.info.origin.position.y + (map_.info.resolution / 2.0);
            path_.poses.push_back(p);
            curr = curr->parent;
        }
        std::reverse(path_.poses.begin(), path_.poses.end());
        RCLCPP_INFO(get_logger(), "A* planned path with %zu waypoints.", path_.poses.size());
    } else {
        RCLCPP_ERROR(get_logger(), "Unable to plan path.");
    }

    // ********
    // * Help *
    // ********
    /*
    Cell cStart(...x-map..., ...y-map...);
    Cell cGoal(...x-map..., ...y-map...);

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);

    openList.push_back(std::make_shared<Cell>(cStart));

    while(!openList.empty() && rclcpp::ok()) {
        ...
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
    */
}

void PlanningNode::smoothPath() {
    // Bezpecnostni podminka kdyby jsme meli jen 2 body, tak je nemame jak narovnavat (kroutit)
    if (path_.poses.size() < 3) return;

    // Inicializace pomocnych promennych (kopirovani cesty)
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    float weight_data = 0.1;
    float weight_smooth = 0.2; // velikost narovnavani trasy
    float tolerance = 0.0001;
    float change = tolerance;
    int iterations = 0;

    // Zastropovani max interaci na 1000
    while (change >= tolerance && iterations < 1000) {
        change = 0.0;
        for (size_t i = 1; i < path_.poses.size() - 1; i++) {   // Zacatek hned za startem a konec pred cilem
            float old_x = newPath[i].pose.position.x;
            float old_y = newPath[i].pose.position.y;
            // Soboj mezi aktualni cestou a kompromisem (cestou mezi dvoma body)
            newPath[i].pose.position.x += weight_data * (path_.poses[i].pose.position.x - newPath[i].pose.position.x) +
                                          weight_smooth * (newPath[i-1].pose.position.x + newPath[i+1].pose.position.x - 2.0 * newPath[i].pose.position.x);
            
            newPath[i].pose.position.y += weight_data * (path_.poses[i].pose.position.y - newPath[i].pose.position.y) +
                                          weight_smooth * (newPath[i-1].pose.position.y + newPath[i+1].pose.position.y - 2.0 * newPath[i].pose.position.y);
            // O kolik se body posunuly
            change += std::abs(old_x - newPath[i].pose.position.x) + std::abs(old_y - newPath[i].pose.position.y);
        }
        iterations++;
    }

    // Nahrazeni zubate cesty smooth
    path_.poses = newPath;
    RCLCPP_INFO(get_logger(), "Path smoothed in %d iterations.", iterations);

    // ********
    // * Help *
    // ********
    /*
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    ... processing ...
    path_.poses = newPath;
    */
}

Cell::Cell(int c, int r) {
    // Inicializace parametru bunky
    x = c;
    y = r;
    f = 0.0;
    g = 0.0;
    h = 0.0;
    parent = nullptr;
}