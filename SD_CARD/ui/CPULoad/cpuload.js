
var proc_2_check = false;


//var xValues = [10,20,30,40,50,60,70,80,90,100];
var xValues = ["0.5","1.0","1.5","2.0","2.5","3.0","3.5","4.0","4.5","5.0","5.5","6.0","6.5","7.0","7.5","8.0","8.5","9.0","9.5","10 "];
$(document).ready(function(){
	

	// websocket : always on
    // --------------------------------------------------------------------------

	function retrieveValueAll(action){
		var ChartValues = new WebSocket("ws://192.168.0.10/ws");
        ChartValues.onmessage = function(event){
			if(!proc_2_check) proc_2_check = true;
            var res = event.data.split("#");
			
			$('.Deb1').text(res[2]);
			$('.Deb2').text(res[3]);
			$('.Deb3').text("WebSockets["+res[5]+","+res[6]+","+res[7]+","+res[8]+","+res[9]+","+res[10]+"]");

			new Chart("myChart", {
				type: "line",
				data: {
				labels: xValues,
					fontColor: "yellow",
					datasets: [{
					label : "ESP32 CORE 0 (Protocol Tasks)",
					data: JSON.parse(res[0]),
					borderColor: "red",
					fill: false
				},{
				label : "ESP32 CORE 1 (Application Tasks)",
				data: JSON.parse(res[1]),
				borderColor: "Blue",
				fill: false
				}]
			},
			options: {
			legend: {display: true,
					 labels: {
                    fontColor: "yellow"
                },
				
			},
            scales: {
                yAxes: [{
					gridLines: {
						display: true,
						color : "green",
						lineWidth : 1,
						},
                    ticks: {
                        fontColor: "yellow",
                        fontSize: 14,
                    }
                }],
                xAxes: [{
					gridLines: {
						display: true,
						color : "green",
						lineWidth : 1,
						},
                    ticks: {
                        fontColor: "yellow",
                        fontSize: 14,
                    },
					
                }],
	
            },
			animation: {duration: 0},
			}
			});


        }
        ChartValues.onopen = function(){
			ChartValues.send(action);
        }
		ChartValues.onclose = function() {
			proc_2_check = false;
		}
		ChartValues.onerror = function() {
			proc_2_check = false;
		}
    }




	setTimeout(function(){
		retrieveValueAll('cpu_stats');
    }, 1000);

	setTimeout(function(){ // auto create connection again in case connection has closed (or connection has failed)
		setInterval(function () {

			if(!proc_2_check) {
				retrieveValueAll('cpu_stats');
			}
		}, 10000);
    }, 60000);

});