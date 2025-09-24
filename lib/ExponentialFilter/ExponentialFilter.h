#ifndef EXPONENTIAL_FILTER_H
#define EXPONENTIAL_FILTER_H

template <typename T>
class ExponentialFilter {
public:
    ExponentialFilter(float alpha, T initial = 0) 
        : alpha(alpha), filtered(initial) {}
    
    T Filter(T new_value) {
        filtered = alpha * new_value + (1 - alpha) * filtered;
        return filtered;
    }
    
    T Current() const { return filtered; }
    
    void SetAlpha(float new_alpha) {
        alpha = new_alpha;
    }
    
    void Reset(T value = 0) {
        filtered = value;
    }

private:
    float alpha;
    T filtered;
};

#endif // EXPONENTIAL_FILTER_H