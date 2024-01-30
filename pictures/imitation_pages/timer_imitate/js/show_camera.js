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

// The ActionClient
// ----------------

var ttsClient = new ROSLIB.ActionClient({
    ros: ros,
    serverName: '/idmind_tabletop/action_tts',
    actionName: 'idmind_tabletop_msgs/TTSAction'
});


totalTime = 4;
timeLeft = 4;
routineId = "";

// Calling a service to trigger a routine


var routineClient = new ROSLIB.Service({
    ros: ros,
    name: '/idmind_tabletop/execute_routine',
    serviceType: 'idmind_tabletop_msgs/Routine'
});

//Publish to /idmind_tabletop/browser_status topic, similar to user_input of PAL Robotics

var bStatusPub = new ROSLIB.Topic({
    ros: ros,
    name: '/idmind_tabletop/browser_action',
    messageType: 'std_msgs/String'
});

var bStatus = new ROSLIB.Message({
    data: 'done'
});


// Take a photo

var pictureClient = new ROSLIB.Service({
    ros: ros,
    name: '/image_cmd',
    serviceType: 'picture_server/SaveImage'
});



function countdown() {
    // Create a goal.


    var goal = new ROSLIB.Goal({
        actionClient: ttsClient,
        goalMessage: {
            message: timeLeft.toString()
        }
    });

    // Print out their output into the terminal.
    goal.on('feedback', function (feedback) {
        console.log('Feedback: ' + feedback);
    });
    goal.on('result', function (result) {
        console.log('Final Result: ' + result);
        document.getElementById("seconds").innerHTML = String(timeLeft);
        if (timeLeft > 0) {
            setTimeout(countdown, 1000);
        } else {
            var goal = new ROSLIB.Goal({
                actionClient: ttsClient,
                goalMessage: {
                    message: "Time is up!"
                }
            });
            goal.send();
            var request = new ROSLIB.ServiceRequest({
                routine: 2000
            });
            routineClient.callService(request, function (result) {
                console.log('Result for service call on '
                    + routineClient.name);

                bStatusPub.publish(bStatus); //publish to the topic that we are done
            });

            console.log(routineId);
            // location.replace("/home/haru/haru_interactions_ws/src/imitation_game/pictures/imitation_pages/gestures_screenshot/gestures_screenshot.html?routine=" + routineId)
        }
        //Take a photo of the user in halfway, cannot do it as service is not reached (but yes for routines above)

        if (timeLeft == Math.round(totalTime / 2, 2)) {
            console.log("Taking photo");
            takePhoto();
        }
    });
    console.log("sending TTS");
    // Send the goal to the action server.
    goal.send();
    timeLeft--;
};


function takePhoto() {
    var request = new ROSLIB.ServiceRequest({
        routine: 2001
    });
    routineClient.callService(request, function (result) {
        console.log('Result for service call on '
            + routineClient.name);

    });
    var goal = new ROSLIB.Goal({
        actionClient: ttsClient,
        goalMessage: {
            message: "Say cheese!"
        }
    });
    //For now dont say cheee only countdown
    //goal.send();
    //Test flash
    $('.flash')
        .show()  //show the hidden div
        .animate({ opacity: 0.5 }, 300)
        .fadeOut(500)
        .css({ 'opacity': 1 });
    //Test taking photo
    var request = new ROSLIB.ServiceRequest({
        cmd: true,
        path: '/home/haru/haru_interactions_ws/src/imitation_game/pictures/imitation_pages/gestures_screenshot/images/',
        num_name: 'user_face'
    });
    pictureClient.callService(request, function (result) {
        console.log('Result for service call on '
            + pictureClient.name);

    });
}
$(document).ready(function () {
    $('.flash').hide();
    const urlParams = new URLSearchParams(window.location.search);
    if (urlParams.has('routine')) {
        const routineValue = urlParams.get('routine');
        $("#haru_img").attr("src", "/home/haru/haru_interactions_ws/src/imitation_game/pictures/imitation_pages/gestures_screenshot/images/routine_" + routineValue + ".png");
    } else {
        $("#haru_img").attr("src", "/home/haru/haru_interactions_ws/src/imitation_game/pictures/question.jpg");
    }
    setTimeout(countdown, 2000);
});


console.log("Time is up!");