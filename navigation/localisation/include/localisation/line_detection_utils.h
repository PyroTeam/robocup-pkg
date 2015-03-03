deplacement_msg::Point convertPtToDeplMsgPt(const Point &point);

std::vector<deplacement_msg::Point> convertPtsToDeplMsgPts(const std::list<Point> &points);

deplacement_msg::Droite convertMdlToDeplMsgDroite(const Modele &modele);

std::vector<deplacement_msg::Droite> convertModelesToDeplMsgDroites(const std::list<Modele> &modeles);

double dist(Point a, Droite d);

Modele ransac(std::list<Point> listOfPoints);

std::list<Modele> findLines(std::list<Point> listOfPoints);