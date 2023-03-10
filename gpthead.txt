#include <iostream>
#include <vector>
#include <fstream>

class Point
{
private:

public:
    Point() {};
    ~Point() {};
    Point(double _x, double _y) {
        x = _x;
        y = _y;
    }
    double x;
    double y;
    int number;
};

class Cloud
{
private:
public:
    int size = 0;
    int number = 0;
    void add_point(Point * point);
    std::vector<double> factors;
    std::vector<Point *> points;
    Cloud(int num) {
        number = num;
    }
};

class Buffer
{
private:
public:
    int points_amount = 0;
    Buffer() {};
    ~Buffer() {};
    std::vector<Point *> points;
    void copy_cloud(Cloud * cloud);
    void rotate(Point center, double phi);
    void scale(Point center, double lambda);
    void translate(std::vector<double> vec);
};

class Field
{
private:
    int points_amount;
    int clouds_amount;
    void add_point(Point * point);
    std::vector<Point *> points;
public:
    //Point vPoint;
    //Cloud vCloud;
    //Buffer vBuffer;
    Field() {
        points_amount = 0;
        clouds_amount = 0;
    };
    ~Field() {};
    std::vector<Point *> get_points();
    int get_amount();
    std::vector<Cloud *> clouds;
    Point* yield_point(int number);
    double distance(Point a, Point b);
    void transform(int cloud_number, int transform_type, double parameter1, double parameter2, double parameter3);
    void generate(int amount, double center_x, double center_y, double deviation_x, double deviation_y);
};

class Interface
{
public:
    //Controller vController;
    //Field vField;
    Interface() {};
    ~Interface() {};
    void start();
};

class Exec
{
private:
    int id;
    int size;
    Field id_field;
public:
    Exec(int _process_id, Field &_field) {
        id = _process_id;
        id_field = _field;
        size = _field.get_amount();
    };
    ~Exec() {};
    std::vector< std::vector <bool> > rclusters;
    void k_means(int k);
    void wave(double delta);
    void DBSCAN(double delta, int k);
    void tree();
    void EM(int k);
    void save();
};

class Controller
{
private:
    std::vector<Exec> processes;
public:
    //Field vField;
    //Exec vExec;
    Controller() {};
    ~Controller() {};
    void save(int process_id);
    void transform(Field &field, int cloud_number, int transform_type, double parameter1, double parameter2, double parameter3);
    void clusterize(Field &field, int process_id, int method, double opt, int opt2);
    void generate(Field &field, int amount, double center_x, double center_y, double deviation_x, double deviation_y);
    void factors(Field &field, int type);
};

class Factors
{
private:
    std::vector<Point *> points;
    int size;
public:
    void clear();
    void add_point(Point * point);
    std::vector<double> calculate();
    Factors() {
        size = 0;
    }
};
