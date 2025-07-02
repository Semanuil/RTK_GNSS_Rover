#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "uart_gnss.h"
#include "imu.h"
#include "motor_control.h"
#include <math.h>

// Navigation constants
#define WAYPOINT_TOLERANCE      2.0     // Meters - distance to consider waypoint reached
#define MAX_SPEED              0.6      // Maximum motor speed (0.0 to 1.0)
#define MIN_SPEED              0.2      // Minimum motor speed for movement
#define TURN_SPEED             0.4      // Speed for turning maneuvers
#define HEADING_TOLERANCE      5.0      // Degrees - heading tolerance for straight movement

// PID controller parameters
#define KP_HEADING             0.8      // Proportional gain for heading control
#define KI_HEADING             0.0      // Integral gain for heading control  
#define KD_HEADING             0.1      // Derivative gain for heading control

#define KP_DISTANCE            0.3      // Proportional gain for distance control
#define KI_DISTANCE            0.0      // Integral gain for distance control
#define KD_DISTANCE            0.05     // Derivative gain for distance control

// Earth radius for distance calculations (meters)
#define EARTH_RADIUS           6371000.0

// Waypoint structure
typedef struct {
    double latitude;     // Decimal degrees
    double longitude;    // Decimal degrees
    float tolerance;     // Acceptance radius in meters
} waypoint_t;

// Navigation command structure
typedef struct {
    float linear_speed;   // Forward/backward speed (-1.0 to 1.0)
    float angular_speed;  // Turning speed (-1.0 to 1.0)
    uint32_t timestamp;   // Command timestamp
} nav_command_t;

// Navigation state
typedef enum {
    NAV_STATE_IDLE,
    NAV_STATE_TURNING,
    NAV_STATE_MOVING,
    NAV_STATE_ARRIVED,
    NAV_STATE_MISSION_COMPLETE
} nav_state_t;

// Rover status structure
typedef struct {
    nav_state_t state;
    int current_waypoint;
    int total_waypoints;
    float distance_to_target;
    float bearing_to_target;
    float current_heading;
    float current_speed;
    double current_lat;
    double current_lon;
} rover_status_t;

// PID controller structure
typedef struct {
    float kp, ki, kd;
    float previous_error;
    float integral;
    uint32_t last_time;
} pid_controller_t;

// External queue handles
extern QueueHandle_t gnss_queue;
extern QueueHandle_t imu_queue;
extern QueueHandle_t nav_queue;

// Global variables
extern waypoint_t *waypoints;
extern int num_waypoints;
extern int current_waypoint_index;
extern nav_state_t navigation_state;

// Function prototypes
esp_err_t navigation_init(waypoint_t *wp_list, int wp_count);
esp_err_t navigation_start(void);
esp_err_t navigation_stop(void);
void navigation_task(void *pvParameters);

// Navigation calculations
float calculate_distance(double lat1, double lon1, double lat2, double lon2);
float calculate_bearing(double lat1, double lon1, double lat2, double lon2);
float normalize_angle(float angle);
float angle_difference(float target, float current);

// PID controller
float pid_update(pid_controller_t *pid, float error, uint32_t current_time);
void pid_reset(pid_controller_t *pid);

// Status functions
rover_status_t get_rover_status(void);

#endif // NAVIGATION_H