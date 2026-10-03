class Variation{
private:
        double time;
        double loadChange;
        double frictionChange;
public:
        Variation(double time, double loadChange, double frictionChange);
        double getTime() const;
        double getLoadChange() const;
        double getFrictionChange() const;
};

