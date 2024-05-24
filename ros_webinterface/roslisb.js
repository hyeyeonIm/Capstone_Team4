$(document).ready(function() {
    // ROS 연결 초기화
    var ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
    });

    // ROS 연결 상태 업데이트
    var statusSpan = document.getElementById("status");
    ros.on('connection', function() {
        console.log('Connected to websocket server.');
        statusSpan.innerHTML = "Connected";

        var roomMarkerListener = new ROSLIB.Topic({
            ros: ros,
            name: '/room_markers',
            messageType: 'visualization_msgs/Marker'
        });

        console.log('Attempting to subscribe to /room_markers');

        roomMarkerListener.subscribe(function(message) {
            console.log('Received message on ' + roomMarkerListener.name + ': ', message);
            displayMarker(message);
        });


    });

    ros.on('error', function(error) {
        console.log('Error connecting to websocket server:', error);
        statusSpan.innerHTML = "Error";
    });

    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        statusSpan.innerHTML = "Closed";
    });


    // 페이지 로드 시 매핑 상태 확인
    if(localStorage.getItem("mappingFinished") === "true") {
        $(".Start_Map, .Stop_Map").hide();
        $(".Mapping_Again, .Segmenatation").show();
    } else {
        $(".Start_Map, .Stop_Map").show();
        $(".Mapping_Again, .Segmenatation").hide();
    }

    // 매핑 시작 버튼 클릭 이벤트
    $(".Start_Map").click(async function() {
        try {
            const data = await $.get("http://localhost:8080/start_hector");
            alert(`Response: ${data}\nStatus: success`);
        } catch (error) {
            console.error("An error occurred:", error);
            alert(`Error starting mapping: ${error.statusText}`);
        }
    });

    // 매핑 중지 및 저장 버튼 클릭 이벤트
    $('.Stop_Map').click(async function() {   
        var mapName = prompt("지도의 이름을 입력해주세요:", "defaultMapName");
        if (mapName) {
            try {
                const saveResponse = await $.get(`http://localhost:8080/stop_and_save/${mapName}`);
                alert(saveResponse); // 저장 완료 알림
    
                const loadResponse = await $.get(`http://localhost:8080/load_map/${mapName}`);
                console.log(loadResponse); // 콘솔에 로딩 결과를 출력
                alert(loadResponse); // 로딩 완료 알림
            } catch (error) {
                console.error("An error occurred:", error);
                alert(`Error: ${error.statusText}`);
            }
        }
        // 매핑 상태 저장
        localStorage.setItem("mappingFinished", "true");
        $(".Start_Map").hide();
        $(".Stop_Map").hide();
        $(".Mapping_Again").show();
        $(".Segmentation").show();
    });

    // 다시 매핑하기 버튼 클릭 이벤트
    $(".Mapping_Again").click(function() {
        // 매핑 상태 초기화 및 저장
        localStorage.setItem("mappingFinished", "true");
        $(".Start_Map").show();
        $(".Stop_Map").show();
        $(this).hide(); // "다시 매핑하기" 버튼 숨기기
        //$(".Segmentation").hide(); // "공간 나누기" 버튼 숨기기
    });

    fetchSavedRooms();

    $('.Segmentation').off('click').on('click', async function() {
        var mapName = prompt("Enter the map name:", "defaultMapName");
        if (mapName) {
            try {
                console.log(`Starting segmentation for map: ${mapName}`);
                const response = await $.get(`http://localhost:8080/Segmentation/${mapName}`);
                console.log(response);
                alert(response);

                // Segmentation 완료 후 room_coordinates.txt 읽기
                console.log("Reading room coordinates");
                const roomResponse = await $.get('http://localhost:8080/room_coordinates.txt');
                console.log("Room coordinates received:", roomResponse);
                updateRoomList(roomResponse);
            } catch (error) {
                console.error("An error occurred:", error);
                alert(`Error: ${error.statusText}`);
            }
        }
    });

    // Save room info button click event
    $('.save_room_info').off('click').on('click', function() {
        var selectedRooms = [];
        document.querySelectorAll('input[name="rooms"]:checked').forEach((checkbox) => {
            selectedRooms.push(checkbox.value);
        });

        if (selectedRooms.length > 0) {
            $.post('http://localhost:8080/save_coordinates', { rooms: selectedRooms.join(',') })
                .done(function(response) {
                    alert("Coordinates saved successfully!");
                    console.log("Coordinates saved:", response);
                    location.reload(); // 화면 새로고침
                })
                .fail(function(error) {
                    console.error("Error saving coordinates:", error);
                    alert("Error saving coordinates.");
                });
        } else {
            alert('No rooms selected');
        }
    });





    // 지도 시각화 관련 코드는 그대로 유지
    var viewer = new ROS2D.Viewer({
        divID: 'map',
        width: 800,
        height: 600
    });

    var gridClient = new ROS2D.OccupancyGridClient({
        ros: ros,
        rootObject: viewer.scene,
        topic: '/map',
        continuous: true
    });

    gridClient.on('change', function() {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });

    function displayMarker(marker) {
        // 선택된 Room ID만 표시
        const markerRoomID = marker.text.replace('Room ', '').trim();
        console.log("Displaying marker for room: ", markerRoomID);
        console.log("Current selected rooms: ", window.selectedRooms);

        if (window.selectedRooms && window.selectedRooms.includes(markerRoomID)) {
            var shape = new createjs.Shape();
            shape.graphics.beginFill("blue").drawCircle(0, 0, 0.1);
            shape.x = marker.pose.position.x;
            shape.y = -marker.pose.position.y;

            viewer.scene.addChild(shape);

            var text = new createjs.Text(marker.text, "0.5px Arial", "#005F73");
            text.x = marker.pose.position.x;
            text.y = -marker.pose.position.y + 0.1;

            viewer.scene.addChild(text);
        } else {
            console.log(`Marker for Room ${markerRoomID} not displayed.`);
        }
    }



// 맵 구독
var mapListener = new ROSLIB.Topic({
    ros: ros,
    name: '/map',
    messageType: 'nav_msgs/OccupancyGrid'
});

mapListener.subscribe(function(message) {
    console.log('Received map data:', message);
    resolution = message.info.resolution;
    mapOriginX = message.info.origin.position.x;
    mapOriginY = message.info.origin.position.y;
});





        // tf 구독
        var tfClient = new ROSLIB.TFClient({
            ros: ros,
            fixedFrame: 'map',
            angularThres: 0.01,
            transThres: 0.01,
            rate: 10.0
        });

        var robotIcon = new Image();
        robotIcon.src = "./icons/buddy_icon.png";

        // 로봇 위치 업데이트 함수
        function updateRobotPosition(x, y, theta) {
            console.log(`Updating robot position: x=${x}, y=${y}, theta=${theta}`); // 위치 업데이트 로그
            var canvas = document.getElementById("robotCanvas");
            var context = canvas.getContext("2d");

            context.clearRect(0, 0, canvas.width, canvas.height);

            context.save();
            context.translate(x, y);
            context.rotate(theta);
            context.drawImage(robotIcon, -robotIcon.width / 2, -robotIcon.height / 2, 20, 20);
            context.restore();
        }

 
        robotIcon.onload = function() {
            tfClient.subscribe('base_footprint', function(tf) {
                console.log('Received tf:', tf); // tf 수신 로그
        
                // 위치 변환 (미터 단위를 픽셀 단위로 변환)
                var x = (tf.translation.x - mapOriginX) / resolution; // 스케일링 조정
                var y = -(tf.translation.y - mapOriginY) / resolution; // 스케일링 조정
        
                // 중심 위치 조정 (맵 중심에 맞추기)
                x += 800; // 적절한 중심 위치 조정 값
                y += 600; // 적절한 중심 위치 조정 값
                
                // 회전 각도 변환 (라디안)
                var siny_cosp = 2.0 * (tf.rotation.w * tf.rotation.z + tf.rotation.x * tf.rotation.y);
                var cosy_cosp = 1.0 - 2.0 * (tf.rotation.y * tf.rotation.y + tf.rotation.z * tf.rotation.z);
                var theta = Math.atan2(siny_cosp, cosy_cosp);
        
                // 로봇 위치 업데이트
                updateRobotPosition(x, y, theta);
            });
        };
        


});

    


    // // Example of subscribing to a topic
    // var moveBaseFeedbackListener = new ROSLIB.Topic({
    //     ros: ros,
    //     name: '/move_base/feedback',
    //     messageType: 'move_base_msgs/MoveBaseActionFeedback'
    // });

    // moveBaseFeedbackListener.subscribe(function(message) {
    //     console.log('Received move_base feedback: ', message);
    //     // Handle or visualize the feedback
    // });

    // Additional functionalities as needed
