var speedTop = 0;
var speedAvg = 0;
var engineSpeedTop = 0;
var totalMoveCount = 0;
var speedSumTotal = 0;
var speedAnimation = 1;

var egt_temp_lowest = 5;
var egt_temp_normal_start = 50;
var egt_temp_normal_end = 150;  // default = 58  *** , high-sense = 56, dangerous-zone = 65
var egt_temp_lvl1 = 230;  // default = 65 , high-sense = 60, dangerous-zone = 70
var egt_temp_lvl2 = 230;  // default = 75 , high-sense = 65, dangerous-zone = 75

var engineTempC = 0;
var engineTempGauge = 0;
var IntakeAirTempC = 0;

var mytimer1;
var mytimer2;
var mytimer1B;
var mytimer2B;

var wsockTimeout = 10000;
var ConnwsockTimeout = 20000;

var speedometerWsValue;
var spDOWebsocket;


var proc_1_check = false;
var proc_2_check = false;
var proc_3_check = false;

$(document).ready(function() {
    // websocket
    // --------------------------------------------------------------------------
  function retrievedata(action) {
	  
	  	clearTimeout(mytimer1B);
		mytimer1B = setTimeout(function(){ 

		sspDOWebsocket.close();
		proc_1_check = false;

		},ConnwsockTimeout);
	  
        spDOWebsocket = new WebSocket("ws://192.168.0.10/ws");
        spDOWebsocket.onmessage = function(event) {
			
			clearTimeout(mytimer1B);
			
			if(!proc_1_check) proc_1_check = true;

			
            var res = event.data.split("#");
			var currentSpeedUse = $.trim(res[0]);	
			speedCurrent = Math.ceil(parseInt(currentSpeedUse) * 1.0 );
			var currentrpmUse = $.trim(res[1]);
			rpmCurrentInt = Math.ceil(parseInt(currentrpmUse) * 1.0 );			
			rpmCurrent = Math.ceil(parseInt(currentrpmUse) * 0.019375 );  // Bar max at 155/8000 rpm 


            updaterpmBar(rpmCurrent);
			$('.txtSpeed').text(speedCurrent);
			$('.txtRpm').text(rpmCurrentInt);
					
			var Throt = res[2];		
			updateThrottlePos(Throt);		

        };
        spDOWebsocket.onopen = function() {
            spDOWebsocket.send(action);
        };
		spDOWebsocket.onclose = function() {
			proc_1_check = false;
		}
        spDOWebsocket.onerror = function(e) {
			proc_1_check = false;
            console.log("err: " + e.toString());
        };
    }
 
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
//			console.log("WS2 Closed");
		}
		speedometerWsValue.onerror = function() {
			speedometerWsValue.close();
			proc_2_check = false;
//			console.log("WS2 Error");
		}
    }

  function updaterpmBar(rpmval) {
       for (var i = 150; i >= 105; i -= 5) {
        var barClassName = '.rpmBar_' + i;
		var barClassNameB = '.rpmBar2_' + i;
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
		  $(barClassNameB).css({ 'background-color': backgroundColor });
        } else {
          $(barClassName).css({ 'background-color': 'transparent' });
		  $(barClassNameB).css({ 'background-color': 'transparent' });
        }
      }
      for (var j = 100; j >= 55; j -= 5) {
        var barClassName2 = '.rpmBar_' + j;
		var barClassName2B = '.rpmBar2_' + j;
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
		  $(barClassName2B).css({ 'background-color': backgroundColor });
        } else {
          $(barClassName2).css({ 'background-color': 'transparent' });
		  $(barClassName2B).css({ 'background-color': 'transparent' });
        }
      }
      for (var k = 50; k >= 5; k -= 5) {
        var barClassName3 = '.rpmBar_' + k;
		var barClassName3B = '.rpmBar2_' + k;
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
		  $(barClassName3B).css({ 'background-color': backgroundColor });
        } else {
          $(barClassName3).css({ 'background-color': 'transparent' });
		  $(barClassName3B).css({ 'background-color': 'transparent' });
        }
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

	setTimeout(function(){
		if(!proc_1_check)
			retrievedata('speed_rpm200');

		if(!proc_2_check)
			retrieveValueAll('all_value3000');

		
    }, 2000);
});






