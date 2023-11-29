var speedCurrent = 0;
var speedTop = 0;

var egt_temp_lowest = 5;
var egt_temp_normal_start = 50;
var egt_temp_normal_end = 150;  // default = 58  *** , high-sense = 56, dangerous-zone = 65
var egt_temp_lvl1 = 230;  // default = 65 , high-sense = 60, dangerous-zone = 70
var egt_temp_lvl2 = 230;  // default = 75 , high-sense = 65, dangerous-zone = 75

var engineTempC = 0;
var engineTempGauge = 0;
var IntakeAirTempC = 0;

var showSpeed = true;


var proc_1_check = false;
var proc_2_check = false;
var proc_3_check = false;

var wsockTimeout = 10000;
var ConnwsockTimeout = 20000;

var chart;
var w;
var h;
var ch;
var cw;
var ch2;
var egt_args = {
	"payload" : {
		"pressType" : "Long",     // "Short", "Long"
		"eventCause" : "Hardkey"    // "Touch", "Multicontroller", "Hardkey"
	}
};

var mytimer1;
var mytimer2;
var mytimer1B;
var mytimer2B;
var speedometerWsVih;
var speedometerWsValue;
var tpmsWebsocketOBD;




var xValues = ["0.5","1.0","1.5","2.0","2.5","3.0","3.5","4.0","4.5","5.0","5.5","6.0","6.5","7.0","7.5","8.0","8.5","9.0","9.5","10 "];


$(document).ready(function(){

	// websocket : always on
    // --------------------------------------------------------------------------

	function retrieveValueAll(action){

		clearTimeout(mytimer2B);
		mytimer2B = setTimeout(function(){ 

				speedometerWsValue.close();
				proc_2_check = false;

		},ConnwsockTimeout);	

		speedometerWsValue = new WebSocket("ws://192.168.0.10/ws");
		
	
					
        speedometerWsValue.onmessage = function(event){
			
			clearTimeout(mytimer2B);
			
			if(!proc_2_check) proc_2_check = true;
            var res = event.data.split("#");

			updateIntakeAirTemp(res[1]);
			updateEngineTemp(res[0]);
			updateCMVoltage(res[4]);
			


			clearTimeout(mytimer2);
			mytimer2 = setTimeout(function(){ 

				speedometerWsValue.close();


				},wsockTimeout);

			
        }
		
        speedometerWsValue.onopen = function(){
			speedometerWsValue.send(action);

        }
		
		speedometerWsValue.onclose = function() {

			proc_2_check = false;

		}
		speedometerWsValue.onerror = function() {

			speedometerWsValue.close();
			proc_2_check = false;

		}
    }

	function retrieveCPU(action){
		var ChartValues = new WebSocket("ws://192.168.0.10/ws");
        ChartValues.onmessage = function(event){
			if(!proc_3_check) proc_3_check = true;
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
			proc_3_check = false;
		}
		ChartValues.onerror = function() {
			proc_3_check = false;
		}
    }

	function retrievedataVihicleSpeedAndRPM(action){
		

		clearTimeout(mytimer1B);
		mytimer1B = setTimeout(function(){ 

				speedometerWsVih.close();
				proc_1_check = false;

		},ConnwsockTimeout);			
		
		speedometerWsVih = new WebSocket("ws://192.168.0.10/ws");


        speedometerWsVih.onmessage = function(event){

			clearTimeout(mytimer1B);

			if(!proc_1_check) proc_1_check = true;
            var res = event.data.split("#");

			var speed = res[0];
			var RPM =  res[1];
			var Throt = res[2];
	

			updateEngineRPM(RPM);
			updateVehicleSpeed(speed);
			updateThrottlePos(Throt);

			clearTimeout(mytimer1);
			mytimer1 = setTimeout(function(){ 

				speedometerWsVih.close();
				
				},wsockTimeout);
			
        }
        speedometerWsVih.onopen = function(){
            speedometerWsVih.send(action);

        }
		speedometerWsVih.onclose = function() {
			proc_1_check = false;

		}
		speedometerWsVih.onerror = function() {
			speedometerWsVih.close();
			proc_1_check = false;

		}
    }


	function updateIntakeAirTemp(IntakeAirTempIn) {
		var needPos = 0;
		var IntakeAirTemp = $.trim(IntakeAirTempIn);
		IntakeAirTemp = parseInt(IntakeAirTemp);

			IntakeAirTempC = Math.round(IntakeAirTemp);

			if(IntakeAirTempC < 5) {
				IntakeAirTempC = 5;
			}
			if(IntakeAirTempC > 60) {
				IntakeAirTempC = 60;
			}
			
			$('.IATDialValue').text(IntakeAirTempC.toString()+" C");

			if (IntakeAirTempC < 35) {
				needPos = Math.round((IntakeAirTempC - 35) * 1.5).toFixed(0);
			} else if (IntakeAirTempC > 45) {
				needPos = Math.round((IntakeAirTempC - 45) * 3).toFixed(0);
			}
			$('.IATNeedle').css("transform","rotate("+needPos+"deg)");

	}

	function updateThrottlePos(ThrottlePosIn) {
		var needPos = 0;
		var ThrottlePos = $.trim(ThrottlePosIn);
		ThrottlePos = parseInt(ThrottlePos);

		ThrottlePosC = Math.round(ThrottlePos);

		$('.IAT2DialValue').text(ThrottlePosC.toString());

		if (ThrottlePosC < 35) {
				needPos = Math.round((ThrottlePosC - 35) * 1.1).toFixed(0);
			} else if (ThrottlePosC > 50) {
				needPos = Math.round((ThrottlePosC - 50) * 1.1).toFixed(0);
			}
		$('.IAT2Needle').css("transform","rotate("+needPos+"deg)");

	}
	
	function updateCMVoltage(CMVoltageIn) {
		var needPos = 0;
		var CMVoltage = $.trim(CMVoltageIn);
		CMVoltage = parseInt(CMVoltage);

		CMVoltageC = Math.round(CMVoltage);

		$('.EGT2DialValue').text(CMVoltageIn+"V");

		if (CMVoltageC < 10) {
				needPos = Math.round((CMVoltageC - 10) * 5.6).toFixed(0);
			} else if (CMVoltageC > 12) {
				needPos = Math.round((CMVoltageC - 12) * 5.6).toFixed(0);
			}
		$('.EGT2Needle').css("transform","rotate("+needPos+"deg)");

	}

	function updateEngineTemp(engineTempIn) {
		var needPos = 0;
		var engineTemp = $.trim(engineTempIn);
		engineTempC = parseInt(engineTemp);
		//engineTempC = Math.round((engineTemp - 32) * 0.556).toFixed(0); // c0nvert to degree C
		engineTempGauge = engineTempC;



		if(engineTempC < egt_temp_lowest) {
			engineTempC = egt_temp_lowest;
		}
		if(engineTempC > egt_temp_lvl2) {
			engineTempC = egt_temp_lvl2;
		}

		$('.EGTDialValue').text(engineTempC.toString()+" C");


        if (engineTempC < egt_temp_normal_start) {
			needPos = Math.round((engineTempC - egt_temp_normal_start)*1.44).toFixed(0);
        } else if (engineTempC > egt_temp_normal_end) {
            //needPos = Math.round((engineTempC - egt_temp_normal_end)*2.647).toFixed(0);  //this is suit for the normal=58, lvl2=75 (17 step)
			needPos = Math.round((engineTempC - egt_temp_normal_end) * (45/(egt_temp_lvl2 - egt_temp_normal_end))).toFixed(0);
		}
        $('.EGTNeedle').css("transform","rotate("+needPos+"deg)");


		if (engineTempC <= egt_temp_normal_end) {
			$('.EGTAlarm').css('opacity','0');

		} else if (engineTempC >= egt_temp_lvl2) {
			$('.EGTAlarm').css('opacity','1');

		} else if (engineTempC > egt_temp_lvl1) {
			$('.EGTAlarm').css('opacity','0.8');

		} else if (engineTempC > egt_temp_normal_end) {
			$('.EGTAlarm').css('opacity','0.4');

		}

		
	}



    // --------------------------------------------------------------------------

    // update vehicle speed
    // --------------------------------------------------------------------------
	function updateVehicleSpeed(currentSpeed){
		var currentSpeedUse = $.trim(currentSpeed);
		speedCurrent = Math.ceil(parseInt(currentSpeedUse) * 1.0 );

		if(speedCurrent > 120) {
			speedCurrent = speedCurrent + 3;
		} else if (speedCurrent > 90) {
			speedCurrent = speedCurrent + 2;
		} else if (speedCurrent > 30) {
			speedCurrent = speedCurrent + 1;
		}


        // update vehicle top speed
        if(speedCurrent > speedTop){
			if(speedCurrent > 180) {
				overscale = (speedCurrent-180)/2;
				$('.topSpeedNeedle').css("transform","rotate("+(-120+180+overscale)+"deg)");
			} else {
				$('.topSpeedNeedle').css("transform","rotate("+(-120+speedCurrent)+"deg)");
			}
			speedTop = speedCurrent;
        }
		$('.speedTopValue').text(speedTop.toString());

		// update vehicle speed
		if(speedCurrent > 180) {
			overscale = (speedCurrent-180)/2;
			$('.speedNeedle').css("transform","rotate("+(-120+180+overscale)+"deg)");
		} else {
			$('.speedNeedle').css("transform","rotate("+(-120+speedCurrent)+"deg)");
		}

		if(showSpeed) {
            $('.txtSpeed').text(speedCurrent.toString());
		}
    }
    // --------------------------------------------------------------------------


	function updateEngineRPM(currentRPM){
		var currentRPMShow = -120 + parseInt((currentRPM * 0.03).toFixed(0));
		$('.rpmNeedle').css("transform","rotate("+currentRPMShow+"deg)");


	}

	setTimeout(function(){
		if(!proc_1_check)
			retrievedataVihicleSpeedAndRPM('speed_rpm200');
		
		if(!proc_2_check)
			retrieveValueAll('all_value3000');
		
		if(!proc_3_check)
		retrieveCPU('cpu_stats');
		
    }, 3000);

	setTimeout(function(){
		
		if(!proc_3_check)
			retrieveCPU('cpu_stats');
	
		
    }, 3000);


 });


