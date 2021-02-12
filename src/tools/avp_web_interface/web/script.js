// taken from http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality
var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});

document.addEventListener("DOMContentLoaded", function() {
    /*
    function setInitialSimPose() {
        var poseTopic = new ROSLIB.Topic({
            ros : ros,
            name : '/localization/initialpose',
            messageType : 'geometry_msgs/msg/PoseWithCovarianceStamped'
        });

        const pose = new ROSLIB.Message({
            header : {
                // stamp should be ignored
                stamp : {
                    sec: 0,
                    nanosec: 0
                },
                frame_id : "map"
            },
            pose : {
                // Initial position in LGSVL
                pose : {
                    position : {
                        x: -57.463,
                        y: -41.644,
                        z: -2.01,
                    },
                    orientation : {
                        x: 0.0,
                        y: 0.0,
                        z: -0.99917,
                        w: 0.04059,
                    },
                },
                // 36 elements
                covariance : [
                    0.25, 0.0,  0.0, 0.0, 0.0, 0.0,
                    0.0,  0.25, 0.0, 0.0, 0.0, 0.0,
                    0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                    0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                    0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                    0.0,  0.0,  0.0, 0.0, 0.0, 0.068,
                ],
            }
        });
        poseTopic.publish(pose);
        console.log('initial sim pose published');
    }

    var initialButton = document.querySelector("[name='initial']");
    initialButton.addEventListener('click', setInitialSimPose);
    */

    function publishGoalPose(pos_x, pos_y, orient_z, orient_w) {
        var goalTopic = new ROSLIB.Topic({
            ros : ros,
            name : '/planning/goal_pose',
            messageType : 'geometry_msgs/msg/PoseStamped'
        });

        // in parsing the pose, 0.0 get converted to an integer leading to problems in publishing the
        // message. So choose something close enough to zero that preserves the float type
        const floatNearZero = 1e-16;

        const pose = new ROSLIB.Message({
            header : {
                // stamp should be ignored
                stamp : {
                    sec: 0,
                    nanosec: 0
                },
                frame_id : "map"
            },
            pose : {
                position : {
                    x: pos_x,
                    y: pos_y,
                    z: floatNearZero, // planning is done in 2D, so z irrelevant
                },
                // park in reverse
                orientation : {
                    x: floatNearZero,
                    y: floatNearZero,
                    z: orient_z,
                    w: orient_w,
                },
            },
        });
        goalTopic.publish(pose);
    }

    function registerCallback(button_name, position, orientation) {
        var button = document.querySelector("[name='" + button_name + "']");
        button.addEventListener('click', () => { publishGoalPose(position.x, position.y, orientation.z, orientation.w); });
    }

    registerCallback("park_forward", {x: -95.875, y: 57.707}, {z: 0.901, w: 0.434});
    registerCallback("park_reverse", {x: -98.56259155273438, y: 60.99168395996094}, {z: -0.42844402469653825, w: 0.9035683248663778});
    registerCallback("return", {x: -26.73, y: 108.795}, {z: 0.342, w: 0.939});
})
