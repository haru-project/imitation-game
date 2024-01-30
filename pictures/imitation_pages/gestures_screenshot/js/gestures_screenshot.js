
// Connecting to ROS
// -----------------
var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function (error) {
    console.log(error);
});

// Find out exactly when we made a connection.
ros.on('connection', function () {
    console.log('Connection made!');

});

ros.on('close', function () {
    console.log('Connection closed.');

});


$(document).ready(function () {
    const urlParams = new URLSearchParams(window.location.search);
    const routineValue = urlParams.get('routine');
    $("#haru_img").attr("src", "/home/haru/haru_interactions_ws/src/imitation_game/pictures/imitation_pages/gestures_screenshot/images/routine_" + routineValue + ".png");
    $("#user_img").attr("src", "/home/haru/haru_interactions_ws/src/imitation_game/pictures/imitation_pages/gestures_screenshot/images/user_face.png");
});