var speedTop = 0;
var speedAvg = 0;
var engineSpeedTop = 0;
var totalMoveCount = 0;
var speedSumTotal = 0;
var speedAnimation = 1;

var proc_2_check = false;


//var xValues = [10,20,30,40,50,60,70,80,90,100];
var xValues = ["0.5","1.0","1.5","2.0","2.5","3.0","3.5","4.0","4.5","5.0","5.5","6.0","6.5","7.0","7.5","8.0","8.5","9.0","9.5","10 "];

$(document).ready(function() {
    // websocket
    // --------------------------------------------------------------------------
  function retrievedata(action) {
        var spDOWebsocket = new WebSocket("ws://192.168.0.10/ws");
        spDOWebsocket.onmessage = function(event) {
            var res = event.data.split("#");
			var currentSpeedUse = $.trim(res[0]);	
			speedCurrent = Math.ceil(parseInt(currentSpeedUse) * 1.0 );
			var currentrpmUse = $.trim(res[1]);
			rpmCurrentInt = Math.ceil(parseInt(currentrpmUse) * 1.0 );			
			rpmCurrent = Math.ceil(parseInt(currentrpmUse) * 0.019375 );  // Bar max at 155/8000 rpm 
			
			if(speedCurrent > 120) {
				speedCurrent = speedCurrent + 3;
			} else if (speedCurrent > 90) {
				speedCurrent = speedCurrent + 2;
			} else if (speedCurrent > 30) {
				speedCurrent = speedCurrent + 1;
			}



                    updaterpmBar(rpmCurrent);
					updateEngineRPM(res[1]);
					$('.txtSpeed').text(speedCurrent);
					$('.txtRpm').text(rpmCurrentInt);

        };
        spDOWebsocket.onopen = function() {
            spDOWebsocket.send(action);
        };
        spDOWebsocket.onerror = function(e) {
            console.log("err: " + e.toString());
        };
    }
 
 
 	function retrieveValueAll(action){
		var ChartValues = new WebSocket("ws://192.168.0.10/ws");
        ChartValues.onmessage = function(event){
			if(!proc_2_check) proc_2_check = true;
            var res = event.data.split("#");
			


			new Chart("myChart", {
				type: "line",
				data: {
				labels: xValues,
					fontColor: "blue",
					datasets: [{
					label : "ESP32 CORE 0 (Proto Hndler Tasks)",
					data: JSON.parse(res[0]),
					borderColor: "red",
					fill: false
				},{
				label : "ESP32 CORE 1 (Applications Tasks)",
				data: JSON.parse(res[1]),
				borderColor: "Lightblue",
				fill: false
				}]
			},
			options: {
			legend: {display: true,
					 labels: {
                    fontColor: "blue"
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
                        fontColor: "blue",
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
                        fontColor: "blue",
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



	function updateEngineRPM(currentRPM){
		var currentRPMShow = -120 + parseInt((currentRPM * 0.03).toFixed(0));
		$('.rpmNeedle').css("transform","rotate("+currentRPMShow+"deg)");

		//$('.txtSpeedUnit').text("rpm = " + currentRPM);

	}
  function updaterpmBar(rpmval) {
       for (var i = 150; i >= 105; i -= 5) {
        var barClassName = '.rpmBar_' + i;
        if (rpmval >= i) {
          switch (i) {
            case 150:
              backgroundColor = '#FF0000';
              break;
            case 145:
              backgroundColor = '#FF0000';
              break;
            case 140:
              backgroundColor = '#FF0000';
              break;
            case 135:
              backgroundColor = '#FF0000';
              break;
            case 130:
              backgroundColor = '#FF0000';
              break;
            case 125:
              backgroundColor = '#FE2E2E';
              break;
            case 120:
              backgroundColor = '#FF451C';
              break;
            case 115:
              backgroundColor = '#FF6932';
              break;
            case 110:
              backgroundColor = '#FE9A2E';
              break;
            case 105:
              backgroundColor = '#FECC20';
              break;
          }
          $(barClassName).css({ 'background-color': backgroundColor });
        } else {
          $(barClassName).css({ 'background-color': 'transparent' });
        }
      }
      for (var j = 100; j >= 55; j -= 5) {
        var barClassName2 = '.rpmBar_' + j;
        if (rpmval >= j) {
          switch (j) {
            case 100:
              backgroundColor = '#FFED2E';
              break;
            case 95:
              backgroundColor = '#FFF430';
              break;
            case 90:
              backgroundColor = '#F7FE2E';
              break;
            case 85:
              backgroundColor = '#C8FE2E';
              break;
            case 80:
              backgroundColor = '#9AFE2E';
              break;
            case 75:
              backgroundColor = '#64FE2E';
              break;
            case 70:
              backgroundColor = '#2EFE2E';
              break;
            case 65:
              backgroundColor = '#2EFE64';
              break;
            case 60:
              backgroundColor = '#2EFE9A';
              break;
            case 55:
              backgroundColor = '#58FAD0';
              break;
          }
          $(barClassName2).css({ 'background-color': backgroundColor });
        } else {
          $(barClassName2).css({ 'background-color': 'transparent' });
        }
      }
      for (var k = 50; k >= 5; k -= 5) {
        var barClassName3 = '.rpmBar_' + k;
        if (rpmval >= k) {
          switch (k) {
            case 50:
              backgroundColor = '#58FAD0';
              break;
            case 45:
              backgroundColor = '#58FAD0';
              break;
            case 40:
              backgroundColor = '#58FAD0';
              break;
            case 35:
              backgroundColor = '#58FAD0';
              break;
            case 30:
              backgroundColor = '#58FAD0';
              break;
            case 25:
              backgroundColor = '#81F7D8';
              break;
            case 20:
              backgroundColor = '#A9F5E1';
              break;
            case 15:
              backgroundColor = '#CEF6EC';
              break;
            case 10:
              backgroundColor = '#E0F8F1';
              break;
            case 5:
              backgroundColor = '#EFFBF8';
              break;
          }
          $(barClassName3).css({ 'background-color': backgroundColor });
        } else {
          $(barClassName3).css({ 'background-color': 'transparent' });
        }
      }
  }


    // Start data retrieval
    setTimeout(function() {
    retrievedata('speed_rpm200');
    }, 2000);
	
	setTimeout(function(){
		retrieveValueAll('cpu_stats');
    }, 1000);
});






