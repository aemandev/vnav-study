#include <cmath> // required for pow and M_PI
#include <iostream>
class Circle{
    double radius;
    public:
        Circle(double rad){
            radius = rad;
        }
        double circumference(){
            return 2*M_PI*radius;
        }
        double area(){
            return M_PI*std::pow(radius,2);
        }
};

int main (){
    Circle circ(3);
    std::cout << "Circumference: " << circ.circumference() << std::endl;
    std::cout << "Area: " << circ.area() << std::endl;
    return 0;
}