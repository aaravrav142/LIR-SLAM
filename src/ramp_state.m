function h = ramp_state()
% simple ramp state class for keeping track of when the robot
% is on/off a ramp

h.upRamp = 1;
h.downRamp = -1;

% initialize to ground
h.state = 0;
h.nlevel = 0;

h.get_state = @get_state;
h.update = @update;


  function state = update(p)
    if abs(p) < 2.5*pi/180
      h.nlevel = h.nlevel + 1;
    else 
      h.nlevel = 0;
    end

    if h.state == 0 && p > 5*pi/180
      %disp('ramp up');
      h.state = 1;
    elseif h.state == 1 && p < -5*pi/180
      %disp('ramp down');
      h.state = -1;
    elseif h.state == -1 && abs(p) < 1.5*pi/180 
      %disp('off ramp');
      h.state = 0;
    elseif h.state == 1 && h.nlevel > 50
      %disp('ramp timeout: off ramp');
      h.state = 0
    end

    state = h.state;
  end

  function s = get_state()
    s = h.state;
  end
end
