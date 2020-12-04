#ifndef STATE_H
#define STATE_H

struct state {
    enum Direction {LEFT, CENTER, RIGHT};
    
    /**
     * @brief Set the polar values for this state
     * 
     * @param distance the distance
     * @param theta the theta (radians)
     * @param direction the direction
     */
    state(double distance=0.0, double theta=0.0, Direction direction=CENTER) 
        {this->distance = distance; this->theta = theta; this->direction = direction;};

    double distance;
    double theta;
    Direction direction;
};

#endif