#include "gpthead.hpp"
#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <cmath>
#include <ctime>
#include <iterator>
#include <algorithm>

#define MAXDST 100
#define EPS 1e-5

void Interface::start() {
    Controller controller;
    Field field;

    std::ifstream input;
    std::ofstream log;

    std::time_t timing = 0;

    std::string buff;

    int fact_type = 1;
    int fact_opt = 0;
    int fact_opt2 = 0;

    int amount = 100;
    int i = 0;
    int method;
    int neighbors;
    double opt;
    int id;
    int cloud_number;
    bool logging = false;
    bool started = false;
    double center_x = 0;
    double center_y = 0;
    double deviation_x = 1;
    double deviation_y = 1;

    int transform_type;
    double transform_center_x;
    double transform_center_y;
    double transform_phi;
    double transform_lambda;
    double transform_vector_x;
    double transform_vector_y;

    input.open("1.txt");

    while(true) {
        if (input.eof()) {
            break;
        }
        input >> buff;
        if (buff == "LOG") {
            input >> logging;
            if (logging) {
                log.open("LOG.txt");
                time(&timing);
                log << "Started " << std::asctime(std::localtime(&timing)) << std::endl;
            }
        } else if (buff == "AMOUNT") {
            input >> amount;
        } else if (buff == "BEGIN") {
            started = true;
            if (logging) {
                time(&timing);
                log << "Starting clusterization " << std::asctime(std::localtime(&timing)) << std::endl;
            }
            controller.clusterize(field, i, method, opt, neighbors);
            i++;
            if (logging) {
                time(&timing);
                log << "Ended clusterization " << std::asctime(std::localtime(&timing)) << std::endl;
            }
        } else if (buff == "GENERATE") {
            if (!started) {
                controller.generate(field, amount, center_x, center_y, deviation_x, deviation_y);
                if (logging) {
                    time(&timing);
                    log << "Generated cloud " << std::asctime(std::localtime(&timing)) << std::endl;
                }
            } else {
                if (logging) {
                    time(&timing);
                    log << "Prevented post-processing generation " << std::asctime(std::localtime(&timing)) << std::endl;
                }
            }
        } else if (buff == "FACTORS") {
            input >> buff;
            if (buff == "FIELD") {
                fact_type = 1;
            } else if (buff == "CLOUDS") {
                fact_type = 2;
            } else {
                fact_type = 3;
            }
            controller.factors(field, fact_type);
            if (logging) {
                time(&timing);
                log << "Calculated factors " << std::asctime(std::localtime(&timing)) << std::endl;
            }
        } else if (buff == "SAVE") {
            input >> id;
            controller.save(id);
            if (logging) {
                time(&timing);
                log << "Saved process with id " << id << " " << std::asctime(std::localtime(&timing)) << std::endl;
            }
        } else if (buff == "TRANSFORM") {
            input >> cloud_number;
            input >> buff;
            if (buff == "ROTATE") {
                transform_type = 0;
                input >> transform_center_x;
                input >> transform_center_y;
                input >> transform_phi;
                controller.transform(field, cloud_number, transform_type, transform_center_x, transform_center_y, transform_phi);
                if (logging) {
                    time(&timing);
                    log << "Rotated cloud " << cloud_number << " " << std::asctime(std::localtime(&timing)) << std::endl;
                }
            } else if (buff == "TRANSLATE") {
                transform_type = 1;
                input >> transform_vector_x;
                input >> transform_vector_y;
                controller.transform(field, cloud_number, transform_type, transform_vector_x, transform_vector_y, 0);
                if (logging) {
                    time(&timing);
                    log << "Translated cloud " << cloud_number << " " << std::asctime(std::localtime(&timing)) << std::endl;
                }
            } else if (buff == "SCALE") {
                transform_type = 2;
                input >> transform_center_x;
                input >> transform_center_y;
                input >> transform_lambda;
                controller.transform(field, cloud_number, transform_type, transform_center_x, transform_center_y, transform_lambda);
                if (logging) {
                    time(&timing);
                    log << "Scaled cloud " << cloud_number << " " << std::asctime(std::localtime(&timing)) << std::endl;
                }
            }
        } else if (buff == "DEVX") {
            input >> deviation_x;
        } else if (buff == "DEVY") {
            input >> deviation_y;
        } else if (buff == "CENTX") {
            input >> center_x;
        } else if (buff == "CENTY") {
            input >> center_y;
        } else if (buff == "END") {
            break;
        } else if (buff == "MODE") {
            input >> buff;
            if (buff == "KMEANS") {
                method = 1;
                input >> opt;
            } else if (buff == "WAVE") {
                method = 2;
                input >> opt;
            } else if (buff == "DBSCAN") {
                method = 3;
                input >> opt;
                input >> neighbors;
            } else if (buff == "EM") {
                method = 4;
                input >> opt;
            } else if (buff == "TREE") {
                method = 5;
            } else {
                if (logging) {
                    time(&timing);
                    log << "Wrong mode " << method << " " << std::asctime(std::localtime(&timing)) << std::endl;
                }
            }
        } else {
            if (logging) {
                time(&timing);
                log << "Wrong parameter " << buff << " " << std::asctime(std::localtime(&timing)) << std::endl;
            }
        }
    }
    input.close();
    log.close();
}



void Controller::clusterize(Field &field, int process_id, int method, double opt, int opt2) {
    Exec process(process_id, field);
    switch (method) {
        case 1:
            process.k_means(int(opt));
            break;
        case 2:
            process.wave(opt);
            break;
        case 3:
            process.DBSCAN(opt, opt2);
            break;
        case 4:
            process.EM(opt);
            break;
        case 5:
            process.tree();
            break;
    }
    processes.push_back(process);
}

void Controller::generate(Field &field, int amount, double center_x, double center_y, double deviation_x, double deviation_y) {
    field.generate(amount, center_x, center_y, deviation_x, deviation_y);
}

void Controller::save(int process_id) {
    processes[process_id].save();
}

void Controller::transform(Field &field, int cloud_number, int transform_type, double parameter1, double parameter2, double parameter3) {
    field.transform(cloud_number, transform_type, parameter1, parameter2, parameter3);
}

void Controller::factors(Field &field, int type) {
    Factors fact;
    std::vector<double> factors;
    std::ofstream out;
    std::ofstream oout;
    std::string filename;

    switch (type) {
        case 1:
            oout.open("FACTORS_OUT.txt");
            out.open("FACTORS_FIELD.txt");
            fact.clear();
            for (Point * point : field.get_points()) {
                fact.add_point(point);
            }
            factors = fact.calculate();
            for (double val : factors) {
                out << val << " ";
            }
            oout << factors[0] << " " << factors[1] << " " << factors[4] << " " << factors[5] << std::endl;
            oout << factors[0] << " " << factors[1] << " " << factors[6] << " " << factors[7] << std::endl;
            out.close();
            oout.close();
            break;
        case 2:
            oout.open("FACTORS_OUT.txt");
            for (int i = 0; i < field.clouds.size(); i++) {
                filename = "FACTORS_CLOUD_" + std::to_string(i) + ".txt";
                out.open(filename);
                fact.clear();
                for (Point * point : (*field.clouds[i]).points) {
                    fact.add_point(point);
                }
                factors = fact.calculate();
                for (double val : factors) {
                    out << val << " ";
                }
                out.close();
                oout << factors[0] << " " << factors[1] << " " << factors[4] << " " << factors[5] << std::endl;
                oout << factors[0] << " " << factors[1] << " " << factors[6] << " " << factors[7] << std::endl;
            }
            oout.close();
            break;
        case 3:
            oout.open("FACTORS_OUT.txt");
            for (int i = 0; i < processes.size(); i++) {
                for (int j = 0; j < processes[i].rclusters.size(); j++) {
                    filename = "FACTORS_CLUSTER_" + std::to_string(i) + "_" + std::to_string(j) + ".txt";
                    out.open(filename);
                    int k = 0;
                    fact.clear();
                    for (bool mark : processes[i].rclusters[j]) {
                        if (mark) {
                            fact.add_point(field.yield_point(k));
                        }
                        k++;
                    }
                    factors = fact.calculate();
                    for (double val : factors) {
                        out << val << " ";
                    }
                    oout << factors[0] << " " << factors[1] << " " << factors[4] << " " << factors[5] << std::endl;
                    oout << factors[0] << " " << factors[1] << " " << factors[6] << " " << factors[7] << std::endl;
                    out.close();
                }
            }
            oout.close();
            break;
    }
}



void Field::generate(int amount, double center_x, double center_y, double deviation_x, double deviation_y) {
    Cloud * cloud = new Cloud(clouds_amount);

    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<double> x_distribution(center_x, deviation_x);
    std::normal_distribution<double> y_distribution(center_y, deviation_y);

    for (int i = 0; i < amount; i++) {
        Point * point = new Point;
        (*point).number = points_amount;
        (*point).x = x_distribution(rd);
        (*point).y = y_distribution(rd);
        add_point(point);
        (*cloud).add_point(point);
        (*cloud).size++;
    }
    clouds.push_back(cloud);
    clouds_amount++;
}

void Field::transform(int cloud_number, int transform_type, double parameter1, double parameter2, double parameter3) {
    Buffer * buffer = new Buffer();
    (*buffer).copy_cloud(clouds[cloud_number]);
    if (transform_type == 0) {
        Point center(parameter1, parameter2);
        (*buffer).rotate(center, parameter3);
    } else if (transform_type == 1) {
        std::vector<double> vec;
        vec.push_back(parameter1);
        vec.push_back(parameter2);
        (*buffer).translate(vec);
    } else if (transform_type == 2) {
        Point center(parameter1, parameter2);
        (*buffer).scale(center, parameter3);
    }
}

int Field::get_amount() {
    return points_amount;
}

double Field::distance(Point a, Point b) {
    return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

void Field::add_point(Point * point) {
    points.push_back(point);
    points_amount++;
}

Point* Field::yield_point(int num) {
    return points[num];
}

std::vector<Point *> Field::get_points() {
    return points;
}



void Cloud::add_point(Point * point) {
    points.push_back(point);
}



void Exec::save() {
    std::string filename;
    std::ofstream out;

    std::vector<Point *> points = id_field.get_points();
    for (long unsigned int i = 0; i < rclusters.size(); i++) {
        filename = "OUT_" + std::to_string(id) + "_" + std::to_string(i) + ".txt";
        out.open(filename);
        for (Point * point : points) {
            int num = (*point).number;
            if(rclusters[i][num]) {
                out << (*point).x << " " << (*point).y << std::endl;
            }
        }
        out.close();
    }
}

void Exec::k_means(int k) {
    std::vector<Point> centers;
    Point center;
    Point fcenter;

    double min_dist;
    double cluster_sum_x;
    double cluster_sum_y;
    int attr;
    int cluster_weight;
    bool differ = false;

    int iters = 0;
    centers.resize(k);

    std::vector<Point *> points = id_field.get_points();
    std::vector< std::vector<bool> > clusters;

    for (int j = 0; j < k; j++) {
        std::vector<bool> cluster;
        for (int i = 0; i < size; i++) {
            cluster.push_back(false);
        }
        clusters.push_back(cluster);
    }

    for (int i = 0; i < k; i++) {
        centers[i] = *(id_field.yield_point(i));
    }

    while (true) {
        differ = false;

        for (int i = 0; i < k; i++) {
            for (int j = 0; j < size; j++) {
                clusters[i][j] = false;
            }
        }

        for (Point * point : points) {
            min_dist = MAXDST;
            for (int clust = 0; clust < k; clust++) {
                if (id_field.distance(*point, centers[clust]) < min_dist) {
                    min_dist = id_field.distance(*point, centers[clust]);
                    attr = clust;
                }
            }
            clusters[attr][(*point).number] = true;
        }

        int clust = 0;
        for (std::vector<bool> cluster : clusters) {
            cluster_weight = 0;
            cluster_sum_x = 0;
            cluster_sum_y = 0;
            for (int i = 0; i < id_field.get_amount(); i++) {
                if (cluster[i]) {
                    Point point = *(id_field.yield_point(i));
                    cluster_sum_x += point.x;
                    cluster_sum_y += point.y;
                    cluster_weight++;
                }
            }
            center = centers[clust];
            center.x = cluster_sum_x/cluster_weight;
            center.y = cluster_sum_y/cluster_weight;
            if (id_field.distance(centers[clust], center) > EPS) {
                differ = true;
            }
            centers[clust] = center;
            clust++;
        }

        if (!differ || iters > 10000) break;
        iters++;
    }
    rclusters = clusters;
}

void Exec::wave(double delta) {
    std::vector<Point *> points = id_field.get_points();
    std::vector<double> matrix;
    std::vector<int> burnt;
    std::vector<bool> cluster;

    std::string filename;
    std::ofstream out;

    bool flag = false;
    int clust;
    int steps = 0;
    int cluster_amount = 0;
    int pamount = id_field.get_amount();

    for (int i = 0; i < pamount; i++) {
        burnt.push_back(0);
    }

    matrix.resize(pamount * pamount);
    for (int i = 0; i < pamount; i++) {
        for (int j = 0; j < pamount; j++) {
            matrix[i * pamount + j] = id_field.distance(*(points[i]), *(points[j]));
        }
    }
    while(true) {
        flag = true;
        for (int i = 0; i < pamount; i++) {
            if (burnt[i] == 0) {
                burnt[i] = 1;
                flag = 0;
                break;
            }
        }
        if (flag) {
            break;
        }

        clust = 0;
        flag = true;
        while (flag) {
            clust++;
            steps++;
            flag = 0;
            for (int i = 0; i < pamount; i++) {
                if (burnt[i] == clust) {
                    for (int j = 0; j < pamount; j++) {
                        if (matrix[i * pamount + j] < delta) {
                            if (burnt[j] == 0) {
                                burnt[j] = burnt[i] + 1;
                                flag = true;
                            }
                        }
                    }
                }
            }
        }
        cluster_amount++;
        for (int i = 0; i < pamount; i++) {
            if (burnt[i] > 0) {
                burnt[i] = -cluster_amount;
            }
        }
    }

    for (int i = 0; i < cluster_amount; i++) {
        cluster.clear();
        for (int k = 0; k < pamount; k++) {
            cluster.push_back(false);
        }
        for (int j = 0; j < pamount; j++) {
            if (burnt[j] == -(i+1)) {
                cluster[j] = true;
            } else {
                cluster[j] = false;
            }
        }
        rclusters.push_back(cluster);
    }

    matrix.clear();
    burnt.clear();
}

void Exec::DBSCAN(double delta, int k) {
    std::vector<Point *> points = id_field.get_points();
    std::vector<double> matrix;
    std::vector<int> neighbors;
    std::vector<int> ncpoint;
    std::vector<int> burnt;
    std::vector<bool> cluster;
    std::vector<int> nncpoints;

    std::string filename;
    std::ofstream out;

    bool flag = false;
    int clust;
    int steps = 0;
    int cluster_amount = 0;
    int pamount = id_field.get_amount();
    double dist;

    for (int i = 0; i < pamount; i++) {
        burnt.push_back(0);
    }

    matrix.resize(pamount * pamount);
    for (int i = 0; i < pamount; i++) {
        ncpoint.push_back(0);
        neighbors.push_back(0);
    }
    for (int i = 0; i < pamount; i++) {
        nncpoints.clear();
        for (int j = 0; j < pamount; j++) {
            dist = id_field.distance(*(points[i]), *(points[j]));
            matrix[i * pamount + j] = dist;
            if (dist < delta) {
                neighbors[i]++;
                neighbors[j]++;
                nncpoints.push_back(j);
                if (neighbors[i] >= k) {
                    ncpoint[i] = 2;
                    for (int point : nncpoints) {
                        if (ncpoint[point] != 2) {
                            ncpoint[point] = 1;
                        }
                    }
                }
            }
        }
    }

    while(true) {
        flag = true;
        for (int i = 0; i < pamount; i++) {
            if ((burnt[i] == 0) && (ncpoint[i] == 2)) {
                burnt[i] = 1;
                flag = 0;
                break;
            }
        }
        if (flag) {
            break;
        }

        clust = 0;
        flag = true;
        while (flag) {
            clust++;
            steps++;
            flag = false;
            for (int i = 0; i < pamount; i++) {
                if ((burnt[i] == clust) && (ncpoint[i] == 2)) {
                    for (int j = 0; j < pamount; j++) {
                        if (matrix[i * pamount + j] < delta) {
                            if (burnt[j] == 0) {
                                burnt[j] = burnt[i] + 1;
                                flag = true;
                            }
                        }
                    }
                }
            }
        }
        cluster_amount++;
        for (int i = 0; i < pamount; i++) {
            if (burnt[i] > 0) {
                burnt[i] = -cluster_amount;
            }
        }
    }

    for (int i = 0; i < cluster_amount; i++) {
        cluster.clear();
        for (int k = 0; k < pamount; k++) {
            cluster.push_back(false);
        }
        for (int j = 0; j < pamount; j++) {
            if (burnt[j] == -(i+1)) {
                cluster[j] = true;
            } else {
                cluster[j] = false;
            }
        }
        rclusters.push_back(cluster);
    }

    out.open("DB_CORE.txt");
    for (int i = 0; i < pamount; i++) {
        if (ncpoint[i] == 2) {
            out << (*points[i]).x << " " << (*points[i]).y << std::endl;
        }
    }
    out.close();
    out.open("DB_NEIGH.txt");
    for (int i = 0; i < pamount; i++) {
        if (ncpoint[i] == 1) {
            out << (*points[i]).x << " " << (*points[i]).y << std::endl;
        }
    }
    out.close();
    out.open("DB_NOISE.txt");
    for (int i = 0; i < pamount; i++) {
        if (ncpoint[i] == 0) {
            out << (*points[i]).x << " " << (*points[i]).y << std::endl;
        }
    }
    out.close();

    matrix.clear();
    burnt.clear();
}

void Exec::EM(int k) {
    std::vector<Point *> points = id_field.get_points();
    int size = points.size();

    std::vector<double> cov(4*k, 0);
    std::vector<double> means(2*k, 1.234);
    for (int i = 0; i < k; i++) {
        means[2*i]=3-i*(4.0/k);
        means[2*i+1]=4-i*(6.0/k);
    }
    std::vector<double> probs(k, 1.0/k);
    std::vector<double> probxc(size*k, 1.0/k);
    std::vector<double> probcx(size*k, 1.0/k);

    std::vector<bool> clusters(k*size, false);
    std::vector<bool> rcluster(size, false);

    double sumX = 0, sumY = 0;
    double osc = 2e-4;
    double eps = 1e-4;
    double power;
    double a, b, m1, m2, c1, c2;
    double dets;
    double P;
    double Pr;
    double denompc;
    double prmax;
    double sumpc, summc;

    std::ofstream out;
    out.open("SCHEISSE.txt");

    int turns = 0;
    int clust;
    int maxturns = 1e4;

    for (Point * point : points) {
        sumX += (*point).x;
        sumY += (*point).y;
    }
    for (int i = 0; i < k; i++) {
        cov[4*i] = sumX*sumX/size;
        cov[4*i+3] = sumY*sumY/size;
    }
    // * норм
    while (osc > eps && turns < maxturns) {
        turns++;
        // ? Расчет матрицы P(x|c)
        for (int c = 0; c < k; c++) {
            dets = cov[4*c]*cov[4*c+3] - cov[4*c+1]*cov[4*c+2];
            //std::cout << dets << std::endl;
            m1 = means[2*c];
            m2 = means[2*c+1];
            for (int x = 0; x < size; x++) {
                a = (*points[x]).x;
                b = (*points[x]).y;
                c1 = a-m1;
                c2 = b-m2;
                power = -0.5 * (c1*(cov[4*c+3]*c1 - cov[4*c+2]*c2) + c2*(cov[4*c]*c2 - cov[4*c+1]*c1))/dets;
                Pr = probxc[size*c + x];
                P = pow(2*M_PI*dets, -0.5) * exp(power);
                //std::cout << P << std::endl;
                probxc[size*c + x] = P;
                if (fabs(Pr - P) > osc) {
                    osc = fabs(Pr - P);
                }
            }
        }
        // ? Расчет матрицы P(c|x)
        for (int c = 0; c < k; c++) {
            for (int x = 0; x < size; x++) {
                denompc = 0;
                for (int cc = 0; cc < k; cc++) {
                    denompc += probxc[size*cc + x] * probs[cc];
                }
                probcx[c*size + x] = (probxc[c*size + x]*probs[c])/denompc;
            }
        }
        /*
        for (int i = 0; i < k*size; i++) {
            if (i % 6 == 0) {
                out << std::endl;
            }
            out << probcx[i] << " ";
        }
        */
        // ? Расчет P(c)
        for (int c = 0; c < k; c++) {
            sumpc = 0;
            for (int x = 0; x < size; x++) {
                sumpc += probcx[size*c + x];
            }

            probs[c] = (1.0/size) * sumpc;
        }
        // ? Расчет центров
        for (int c = 0; c < k; c++) {
            means[2*c] = 0;
            means[2*c+1] = 0;
            for (int x = 0; x < size; x++) {
                means[2*c] += (probcx[c*size + x]/(size*probs[c]))*(*points[x]).x;
                means[2*c+1] += (probcx[c*size + x]/(size*probs[c]))*(*points[x]).y;
            }

            //sout << means[2*c] << " " << means[2*c+1] << std::endl;
        }
        // ? Расчет ковариаций
        for (int c = 0; c < k; c++) {
            m1 = means[2*c];
            m2 = means[2*c+1];
            cov[4*c] = 0;
            cov[4*c+1] = 0;
            cov[4*c+2] = 0;
            cov[4*c+3] = 0;
            for (int x = 0; x < size; x++) {
                a = (*points[x]).x;
                b = (*points[x]).y;
                cov[4*c] += (probcx[size*c + x]/(size*probs[c]))*(a-m1)*(a-m1);
                cov[4*c+1] += (probcx[size*c + x]/(size*probs[c]))*(a-m1)*(b-m2);
                cov[4*c+3] += (probcx[size*c + x]/(size*probs[c]))*(b-m2)*(b-m2);
            }
            cov[4*c+2] = cov[4*c+1];
        }
        //std::cout << turns << std::endl;
    }
    for (int x = 0; x < size; x++) {
        prmax = 0;
        clust = 0;
        for (int c = 0; c < k; c++) {
            if (probcx[c*size + x] > prmax) {
                prmax = probcx[c*size + x];
                clust = c;
            }
        }
        clusters[size*clust + x] = true;
    }

    for (int cluster = 0; cluster < k; cluster++) {
        rcluster.clear();
        for (int x = 0; x < size; x++) {
            rcluster.push_back(clusters[size*cluster + x]);
        }
        rclusters.push_back(rcluster);
    }
    out.close();
}

void Exec::tree() {
    std::vector<Point *> points;
    std::vector<Point *> tree;
    std::ofstream out;
    std::ofstream hist;
    std::ofstream mout;
    out.open("TREE.txt");
    hist.open("HIST.txt");
    points = id_field.get_points();
    int to_check = points.size() - 1;

    double min = 10e5;
    double dist;
    int choice;
    double treeX, treeY, pX, pY;
    std::vector<bool> used;
    for (int i = 0; i < points.size(); i++) {
        used.push_back(false);
    }

    tree.push_back(points[0]);
    used[0] = true;
    int m = 0;
    std::string filename;
    while (to_check > 0) {
        filename = "TREE_" + std::to_string(m) + ".txt";
        mout.open(filename);
        choice = 0;
        min = 10e5;
        for (Point * branch : tree) {
            for (int i = 0; i < points.size(); i++) {
                if (used[i] == 0) {
                    dist = sqrt(((*points[i]).x - (*branch).x)*((*points[i]).x - (*branch).x) + ((*points[i]).y - (*branch).y)*((*points[i]).y - (*branch).y));
                    if (dist < min) {
                        min = dist;
                        choice = i;
                        treeX = (*branch).x;
                        treeY = (*branch).y;
                        pX = (*points[i]).x;
                        pY = (*points[i]).y;
                    }
                }
            }
        }
        tree.push_back(points[choice]);
        used[choice] = 1;
        hist << min << std::endl;
        out << treeX << " " << treeY << std::endl << pX << " " << pY << std::endl;
        mout << treeX << " " << treeY << std::endl << pX << " " << pY << std::endl;
        out << std::endl;
        to_check--;
        mout.close();
        m++;
    }
    out.close();
    hist.close();
}


void Buffer::translate(std::vector<double> vec) {
    for (Point * point : points) {
        (*point).x = (*point).x + vec[0];
        (*point).y = (*point).y + vec[1];
    }
}

void Buffer::copy_cloud(Cloud * cloud) {
    points.clear();
    for (Point * point : (*cloud).points) {
        points.push_back(point);
        points_amount++;
    }
}

void Buffer::scale(Point center, double lambda) {
    std::vector<double> vec;
    vec.resize(2);
    vec[0] = -center.x;
    vec[1] = -center.y;
    translate(vec);
    for (Point * point : points) {
        (*point).x *= lambda;
        (*point).y *= lambda;
    }
    vec[0] = center.x;
    vec[1] = center.y;
    translate(vec);
}

void Buffer::rotate(Point center, double phi) {
    double _x, _y;
    double x, y;
    std::vector<double> vec;
    vec.resize(2);
    vec[0] = -center.x;
    vec[1] = -center.y;
    translate(vec);
    for (Point * point : points) {
        x = (*point).x;
        y = (*point).y;
        _x = x*cos(phi*M_PI/180) - y*sin(phi*M_PI/180);
        _y = x*sin(phi*M_PI/180) + y*cos(phi*M_PI/180);
        (*point).x = _x;
        (*point).y = _y;
    }
    vec[0] = center.x;
    vec[1] = center.y;
    translate(vec);
}



void Factors::add_point(Point * point) {
    points.push_back(point);
    size++;
}

void Factors::clear() {
    points.clear();
    size = 0;
}

std::vector<double> Factors::calculate() {
    std::vector<double> ans;
    std::vector<double> matrix;
    std::vector<double> X;

    double eigenvalue1;
    double eigenvalue2;

    std::vector<double> eigenvector1;
    std::vector<double> eigenvector2;

    X.resize(2*size);
    matrix.resize(4);

    int i = 0;

    double meanX = 0, meanY = 0;

    for (Point * point : points) {
        meanX += (*point).x;
        meanY += (*point).y;
    }

    meanX /= size;
    meanY /= size;

    for (Point * point : points) {
        X[i] = (*point).x - meanX;
        X[i+size] = (*point).y - meanY;
        i++;
    }

    for (int j = 0; j < size; j++) {
        matrix[0] += X[j]*X[j];
        matrix[1] += X[j]*X[j+size];
        matrix[2] += X[j]*X[j+size];
        matrix[3] += X[j+size]*X[j+size];
    }

    eigenvalue1 = (matrix[0]+matrix[3])/2 + sqrt((matrix[0]+matrix[3])*(matrix[0]+matrix[3])/4 - matrix[0]*matrix[3]+matrix[1]*matrix[2]);
    eigenvalue2 = (matrix[0]+matrix[3])/2 - sqrt((matrix[0]+matrix[3])*(matrix[0]+matrix[3])/4 - matrix[0]*matrix[3]+matrix[1]*matrix[2]);

    eigenvector1.push_back(((eigenvalue1-matrix[3])/matrix[2])/sqrt(((eigenvalue1-matrix[3])/matrix[2])*((eigenvalue1-matrix[3])/matrix[2])+1));
    eigenvector2.push_back(((eigenvalue2-matrix[3])/matrix[2])/sqrt(((eigenvalue2-matrix[3])/matrix[2])*((eigenvalue2-matrix[3])/matrix[2])+1));
    eigenvector1.push_back(1/sqrt(((eigenvalue1-matrix[3])/matrix[2])*((eigenvalue1-matrix[3])/matrix[2])+1));
    eigenvector2.push_back(1/sqrt(((eigenvalue2-matrix[3])/matrix[2])*((eigenvalue2-matrix[3])/matrix[2])+1));

    ans.push_back(meanX);
    ans.push_back(meanY);
    ans.push_back(eigenvalue1);
    ans.push_back(eigenvalue2);
    ans.push_back(eigenvector1[0]);
    ans.push_back(eigenvector1[1]);
    ans.push_back(eigenvector2[0]);
    ans.push_back(eigenvector2[1]);

    return ans;
}
