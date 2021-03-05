% ======================================== %
% === MATLAB GUI FOR SPYDERCAM TEAM 18 === %
% ======================================== %

% === Main Program === %
clear;

% --- Variables --- %
global drawing_input; %Cell array holding sets of user drawn lines
drawing_input = cell(1);

global num_sets;   %Number of user entered sets of lines. 
num_sets = 0;

global home;       %Home Location for Payload
home = [0 0 3];

global draw_height; %Height (Z Pos) for Drawing with Pen attachment
draw_height = 3.6;

global non_draw_height; %Height (Z Pos) for NOT Drawing with pen
non_draw_height = 4.6;

global gcode_buffer; %Buffer for holding G-Code to be sent to Arduino
gcode_buffer = [];

global sensor_data; %Incoming sensor data from the Arduino is stored here
sensor_data = zeros(0,5);

global serial_on;   %Turn on or off incoming serial (1 = on, 0 = off)
serial_on = 1;

global prgm_end;    %Set to 0 if the current Gcode sequence is over
prgm_end = 1;

% --- Serial --- %
global s;   %Serial port. Throw an error and quit if it wont open
try s = serialport("COM3",9600);
catch
    disp("Serial device not connected. Reconnect and restart");
    return;
end
% Callback function for serial. Executes if there is 1 byte available
configureCallback(s,"byte",1,@readSerialData) 

% --- GUI Figure --- %
global gui; %GUI uifigure and overarching labels
gui = uifigure(1,'Position', [250, 70, 1100, 700],'Name','SpyderCam Team 18 MATLAB GUI');

% Main area labels
drw_lbl = uilabel(gui,'Text', 'Drawing Input', 'Position',[45 660 200 40],'FontSize',24);
gcode_lbl = uilabel(gui,'Text', 'G-Code Input', 'Position',[400 660 200 40],'FontSize',24);
ea_lbl = uilabel(gui,'Text', 'Execution Area', 'Position',[740 660 200 40],'FontSize',24);

% Child figures
global sensorViewFig; %Figure that shows sensor data
global bufferfig;   %Figure that shows Gcode buffer

% --- User Drawing Input Area --- %
draw_area = uiaxes(gui,'Position',  [20, 227, 340, 440],'ButtonDownFcn',@axesCallback); %Create Axes
draw_area.XLim = [0 8.5]; %Set axes limits
draw_area.YLim = [0 11];
draw_area.PickableParts = 'all'; %Make axes clickable
draw_area.HitTest = 'off'; %Make axes not clickable
draw_area.NextPlot = 'replacechildren'; %Each plot replaces the last

% Buttons for adding a new line set or clearing the drawing
draw_line = uibutton(gui,'state','Text','Draw Line','Position',[30, 197, 100, 22],'ValueChangedFcn',{@drawBtnCallback,draw_area}); %State button: pushed in is drawing mode, pushed out = finished drawing line set
clear_drawing = uibutton(gui,'push','Text','Clear Drawing','Position',[140, 197, 100, 22],'ButtonPushedFcn',{@clearBtnCallback,draw_area}); %Push button: clears everything in drawing window
slbl = uilabel(gui,'Text','Speed (In/s)','Position',[25, 167, 70, 25]); %Label for Speed Slider
speed = uislider(gui,'Position',[110, 230, 180, 3],'Limits',[0.1 4.5]);  %Speed Slider
convert_to_gcode = uibutton(gui,'push','Text','Convert to G-Code ->','Position',[250, 197, 110, 22]); %Convert to G-Code Button


% --- GCODE Input Area --- %
gcode_area = uitextarea(gui,'Position',[400, 270, 300, 390]); %Text box for inputting GCODE
gcode_area.Value = sprintf(";"); %Initialize gcode text input area
gcode_dropdown = uidropdown(gui,'Items',{'G00','G01','G04','G20','G21','G28','G90','G91','M00','M02','M06','M30','M72'},'Position',[400,230,100,22],'ValueChangedFcn',@GdownCallback); %Dropdown for choosing G-code
arg_boxes(); %Set up boxes for inputting arguments
disp_args('G00','000'); %Initialize argument boxes with the ones for G00
global currentgcode; %Current G-Code being formulated with dropdown and argument input boxes
enter_gcode = uibutton(gui,'push','Text','Enter G-Code','Position',[400, 190, 100, 22],'ButtonPushedFcn',{@enterCallback,gcode_area}); %Send current G-Code into the G-Code text area
line_break = uibutton(gui,'push','Text','Line Break','Position',[520, 190, 100, 22],'ButtonPushedFcn',{@linebreakCallback,gcode_area}); %Add a line break (;)
delete_line = uibutton(gui,'push','Text','Delete Line','Position',[400, 150, 100, 22],'ButtonPushedFcn',{@deleteLineCallback,gcode_area});%Delete 1 line from the G-Code text area
clear_gcode = uibutton(gui,'push','Text','Clear G-Code','Position',[520, 150, 100, 22],'ButtonPushedFcn',{@clearGcodeCallback,gcode_area}); %Clear all G-Code from text area

convert_to_gcode.ButtonPushedFcn = {@convertCallback, gcode_area,speed}; %Set callback for convert_to_gcode button. Down here so it can reference speed
                                                                         

% --- Execution Area --- %
incr_distance = uieditfield(gui,'Position',[800, 270, 50, 20]); %Incremental movement distance
incr_dlbl = uilabel(gui,'Text','Distance','Position',[750, 270, 60, 25]); %label for ^
incr_speed = uieditfield(gui,'Position',[940, 270, 50,20]); %Incremental movement speed
incr_slbl = uilabel(gui,'Text','Speed (In/s)','Position',[865, 270, 70,25]); %Label for ^

% X, Y, and Z incremental movement buttons
yplus_btn = uibutton(gui,'push','Text','Y+','Position',[850, 530, 100, 100],'ButtonPushedFcn',{@directionBtnCallback,"Y+",incr_distance,incr_speed});
xminus_btn = uibutton(gui,'push','Text','X-','Position',[740, 420, 100, 100],'ButtonPushedFcn',{@directionBtnCallback,"X-",incr_distance,incr_speed});
xplus_btn = uibutton(gui,'push','Text','X+','Position',[960, 420, 100, 100],'ButtonPushedFcn',{@directionBtnCallback,"X+",incr_distance,incr_speed});
yminus_btn = uibutton(gui,'push','Text','Y-','Position',[850, 310, 100, 100],'ButtonPushedFcn',{@directionBtnCallback,"Y-",incr_distance,incr_speed});
zplus_btn = uibutton(gui,'push','Text','Z+','Position',[960, 530, 50, 50],'ButtonPushedFcn',{@directionBtnCallback,"Z+",incr_distance,incr_speed});
zminus_btn = uibutton(gui,'push','Text','Z-','Position',[790, 360, 50, 50],'ButtonPushedFcn',{@directionBtnCallback,"Z-",incr_distance,incr_speed});
home_btn = uibutton(gui,'push','Text','Home','Position',[850, 420, 100, 100],'ButtonPushedFcn',{@directionBtnCallback,"home",incr_distance,incr_speed}); %Button for sending the payload to the home locations

% Buffer and sensor data control buttons
send_btn = uibutton(gui,'push','Text','Send G-Code to Buffer (From Text Area)','Position',[740, 150, 270, 50],'ButtonPushedFcn',{@send2bufferCallback,gcode_area}); %Sends G-Code from text area to buffer
run_btn = uibutton(gui,'state','Text','Run','Position',[1020, 150, 40, 50],'ValueChangedFcn',@runCallback,'Tag','RUN'); %Start the gcode sequence
sensor_data_btn = uibutton(gui,'push','Text', 'Sensor Data','Position',[740, 210, 100, 20],'ButtonPushedFcn',@sensorViewCallback); %Show the sensor data window
buffer_btn = uibutton(gui,'push','Text', 'G-Code Buffer','Position',[845, 210, 100, 20],'ButtonPushedFcn',@bufferCallback); % Displays the contents of the buffer in a new window
clear_buffer_btn = uibutton(gui,'push','Text','Clear Buffer (0)','Position',[950, 210, 110, 20],'Tag','C','ButtonPushedFcn',@bufferClearCallback); %Clears the contents of the buffer (if they haven't been sent to the Arduino)

% Direct Serial Area
ds_lbl = uilabel(gui,'Text','Direct Serial Comm','Position',[40, 100, 300, 40],'FontSize',24); % Label for direct serial area
ser_get = uitextarea(gui,'Position',[30 50 950, 50],'Tag','SER','ValueChangedFcn',@serGetCallback); % Box for receiving serial
dir_ser = uitextarea(gui,'Position',[30 20 950, 20]); % Box for sending serial
dir_send = uibutton(gui,'push','Text','Send','Position',[990 20 70, 80],'ButtonPushedFcn',{@dirSendCallback,dir_ser,ser_get}); % Button that sends serial

% === UTILITY FUNCTIONS === %
function plot_input(obj,new_point)
    % This is the plotting function. It takes in the axes object and a new
    % point. It adds the point to drawing input, then plots all of drawing
    % input. Inputing [-1 -1] clears the plot
    
    global drawing_input;
    global num_sets;
    
    if new_point ~= [-1 -1] %Only plot if the new point is not [-1 -1];
        drawing_input{num_sets+1} = [drawing_input{num_sets+1}; new_point]; %Add new point to drawing input
        
        inplot = cell(num_sets+1); %Input plot
        obj.NextPlot = 'add'; % Change plot settings so that plots get added to existing plots
        for i = 1:1:num_sets+1 % Draw each cell (1 cell = 1 line set) of drawing input. Make them not clickable.
            inplot{i} = plot(obj,drawing_input{i}(:,1),drawing_input{i}(:,2),'r-o'); 
            inplot{i}.HitTest = 'off';
        end
        obj.NextPlot = 'replacechildren'; %Change plot back to replaceable
    else %If new_point is [-1 -1]
        plot(obj,-1,-1); %Just plot 1 garbage point off the axes
    end
end

function disp_args(code,prevcode)
    % This is the display arguments function. This function turns on or off
    % the visibility of each argument editfield depending on which G-Code
    % has been chosen in the dropdown menu. code and prevcode refer to the
    % new and old values of the dropdown, respectively.
    
    global gui;
    global currentgcode;
    children = get(gui,'Children');
    
    % If dropdown changed from G00
    if sum(prevcode == 'G00') == 3
        %Find elements with the G00 tag and make them invisible. 
        for i = 1:1:length(children)
            if ~isempty(children(i).Tag) && (children(i).Tag == "G00")
                children(i).Visible = 'off';
                %Find the editfield and set its value to empty
                if length(children(i).Type) == 11
                    children(i).Value = '';
                end
            end
        end
    %Find the editfields/labels with the G01 or G00 tag and make them
    %invisible. (G00 and G01 both have XYZ, G01 just also has F)
    elseif sum(prevcode == 'G01') == 3
        for i = 1:1:length(children)
            if ~isempty(children(i).Tag) && ((children(i).Tag == "G00") || (children(i).Tag == "G01"))
                children(i).Visible = 'off';
                %Find the editfield and set its value to empty
                if length(children(i).Type) == 11
                    children(i).Value = '';
                end
            end
        end
    %Find the editfields/labels with the G04 tag and make them invisible
    elseif sum(prevcode == 'G04') == 3
        for i = 1:1:length(children)
            if ~isempty(children(i).Tag) && (children(i).Tag == "G04")
                children(i).Visible = 'off';
                %Find the editfield and set its value to empty
                if length(children(i).Type) == 11
                    children(i).Value = '';
                end
            end
        end
    else
        
    end
    
    %If the new code is G00, turn editfields/labels tagged G00 visible
    if sum(code == 'G00') == 3
        for i = 1:1:length(children)
            if ~isempty(children(i).Tag) && (children(i).Tag == "G00")
                children(i).Visible = 'on';
            end
        end
        %Set the current G-Code to be filled with arguments
        currentgcode = ["G00" "" "" ""];
    %If new code is G01, turn editfields/labels tagged G00 or G01 visible
    elseif sum(code == 'G01') == 3
        for i = 1:1:length(children)
            if ~isempty(children(i).Tag) && ((children(i).Tag == "G00") || (children(i).Tag == "G01"))
                children(i).Visible = 'on';
            end
        end
        %Set the current G-Code to be filled with arguments
        currentgcode = ["G01" "" "" "" ""];
    %If the new code is G04, turn editfields/labels tagged G04 visible
    elseif sum(code == 'G04') == 3
        for i = 1:1:length(children)
            if ~isempty(children(i).Tag) && (children(i).Tag == "G04")
                children(i).Visible = 'on';
            end
        end
        %Set the current G-Code to be filled with arguments
        currentgcode = ["G04" ""];
    else
        %Set the current G-Code (no arguments)
        currentgcode = [convertCharsToStrings(code)];
    end
end

function arg_boxes()
    % This function sets up the editfields and labels for argument input on
    % the G-Code Input. It puts all of the items in position, then makes
    % them all invisible

    global gui;
    
    % XYZ labels/editfields
    xl = uilabel(gui,'Text', 'X','Position',[520 230 10 20],'Tag',"G00");
    xe = uieditfield(gui,'Position',[535 230 30 20],'Tag',"G00",'ValueChangedFcn',{@fieldChange,'X','0'});
    yl = uilabel(gui,'Text', 'Y','Position',[580 230 10 20],'Tag',"G00");
    ye = uieditfield(gui,'Position',[595 230 30 20],'Tag',"G00",'ValueChangedFcn',{@fieldChange,'Y','0'});
    zl = uilabel(gui,'Text', 'Z','Position',[640 230 10 20],'Tag',"G00");
    ze = uieditfield(gui,'Position',[655 230 30 20],'Tag',"G00",'ValueChangedFcn',{@fieldChange,'Z','0'});
    
    % F label/editfield
    fl = uilabel(gui,'Text', 'F','Position',[640 190 10 20],'Tag',"G01");
    fe = uieditfield(gui,'Position',[655 190 30 20],'Tag',"G01",'ValueChangedFcn',{@fieldChange,'F','0'});
    
    % PX dropdown/editfield
    pxdd = uidropdown(gui,'Items',{'P','X'},'Position',[520 230 40 20],'Tag',"G04");
    pxe = uieditfield(gui,'Position',[565 230 30 20],'Tag',"G04",'ValueChangedFcn',{@fieldChange,'P',pxdd});
    pxdd.ValueChangedFcn = {@pxChange,pxe};
    
    % Turn all of the elements created in this function invisible
    xl.Visible = 'off';
    xe.Visible = 'off';
    yl.Visible = 'off';
    ye.Visible = 'off';
    zl.Visible = 'off';
    ze.Visible = 'off';
    fl.Visible = 'off';
    fe.Visible = 'off';
    pxdd.Visible = 'off';
    pxe.Visible = 'off';
    
end

function [gcode] = drawing2gcode(speed)
    % This function turns data from the user inputted drawing and turns it
    % into G-Code. It takes in a constant speed for the whole operation. It
    % orders the drawing lines by proximity and then turns that path into
    % G-Code

    %Setup
    global draw_height;
    global non_draw_height;
    
    F = speed;
    global home;
    global drawing_input;
    sets = length(drawing_input);
    if length(drawing_input{sets}) < 1
       sets = sets-1;
    end
    indexes = [];
    
    %Find the line set that starts closest to the origin
    lowest_dist = 100;
    for i = 1:1:sets
       temp = sqrt(sum((home(1:2) - drawing_input{i}(1,:)).^2));
       if temp < lowest_dist
            lowest_dist = temp;
            indexes(1) = i;
       end
    end
    
    %Add line set that starts closest to the origin to a new ordered array
    new_order{1} = drawing_input{indexes(1)};
    
    % Order the rest of of the line sets, next closest starting point after
    % current set endpoint.
    while length(new_order) < sets
    lowest_dist = 100;
        for i = 1:1:sets
            if ~ismember(i,indexes)
                temp = sqrt(sum((new_order{end}(end,:) - drawing_input{i}(1,:)).^2));
                if temp < lowest_dist
                    lowest_dist = temp;
                    indexes(length(indexes)+1) = i;
                    new_order{length(new_order)+1} = drawing_input{i};
                end
            end
        end
    end
    
    %Initialize gcode
    gcode = {};
    gcode{1} = sprintf(';');
     
    % Turn each set into a set of G-Code Commands
    for i = 1:1:sets
        %Go to location of first point and lower down into pen height
        gcode{length(gcode)+1} = ['G00' ' X' num2str(new_order{i}(1,1)) ' Y' num2str(new_order{i}(1,2)) ' Z' num2str(non_draw_height) ';'];
        gcode{length(gcode)+1} = ['G01' ' Z' num2str(draw_height) ' F' num2str(F) ';'];
        %Stay in pen heigh and go through the points in the set (draw the
        %line)
        for j = 2:1:length(new_order{i})
            gcode{length(gcode)+1} = ['G01' ' X' num2str(new_order{i}(j,1)) ' Y' num2str(new_order{i}(j,2)) ' F' num2str(F) ';'];
        end
        %Lift up out of pen height
        gcode{length(gcode)+1} = ['G01' ' Z3' ' F' num2str(F) ';'];
    end
    
end
% ====================================== %



% === GUI ELEMENT CALLBACK FUNCTIONS === %
function axesCallback(obj,~)
    % This is the callback function for clicking on the Drawing Axes.
    % It gets the mouse click location and sends it to the plotting
    % function.
    
    mouse_loc = obj.CurrentPoint;
    plot_input(obj,mouse_loc(1,1:2));
end

function drawBtnCallback(obj,~,obj2)
    % This is the callback function for clicking the "Draw Line" Button.
    % It turns on the ability to draw on the axes.
    
    global drawing_input;
    global num_sets;
    
    if obj.Value == 1                   % If button is pressed
        obj2.HitTest = 'on';                % Turn axis clicking on
        obj.Text = 'End Line';
    else                                % If button is not pressed (un-pressed? pulled?)
        obj2.HitTest = 'off';               % Turn axis clicking off
        obj.Text = 'Draw Line';
        if ~isempty(drawing_input{num_sets+1}) % If something has been drawn in this set
            num_sets = num_sets + 1;            % Add 1 to num_sets
            drawing_input{num_sets+1} = [];        % Add another cell to user input
        end
    end
end

function clearBtnCallback(~,~,obj2)
    % This is the callback function for the "Clear Drawing" button.
    % It just resets the user input/number of sets then plots an empty
    % graph.
    
    global drawing_input;
    global num_sets;
    
    num_sets = 0;
    drawing_input = cell(1);
    
    plot_input(obj2,[-1 -1]);
    
end

function GdownCallback(obj,event)
    %This is the callback function for the G-Code dropdown menu. It grabs
    %the current and previous values, then sends them to the disp_args
    %function
    
    val = obj.Value;
    prevVal = event.PreviousValue;
    disp_args(val,prevVal);
    
end

function enterCallback(~,~,text_area)
    %This is the callback function for the send gcode button. It adds the
    %G-Code formed by the dropdown and argument inputs to the text area.

    global currentgcode;
    index = length(text_area.Value)+1;
    text_area.Value{index} = sprintf('');
    for i = 1:1:length(currentgcode)
        charcode = convertStringsToChars(currentgcode(i));
        if ~isempty(charcode)
            text_area.Value{index} = [text_area.Value{index} sprintf(charcode)  sprintf(' ')];
        end
    end
    text_area.Value{index} = text_area.Value{index}(1:end-1);
    text_area.Value{index} = [text_area.Value{index} sprintf(';')];
    
end

function fieldChange(obj,~,lbl,obj2)
    %This is the callback function for when the argument fields are
    %updated. This function runs every time those are changed values.

    global currentgcode;
    
    %Depending on the argument label, add argument to current g code
    if lbl == 'X'
        currentgcode(2) = convertCharsToStrings([lbl obj.Value]);
    elseif lbl == 'Y'
        currentgcode(3) = convertCharsToStrings([lbl obj.Value]);
    elseif lbl == 'Z'
        currentgcode(4) = convertCharsToStrings([lbl obj.Value]);
    elseif lbl == 'F'
        currentgcode(5) = convertCharsToStrings([lbl obj.Value]);
    elseif lbl == 'P'
        currentgcode(2) = convertCharsToStrings([obj2.Value obj.Value]);
    else
        
    end
  
end

function convertCallback(~,~,text_area,speed)
    % This is the callback function for the convert to G-Code button. It
    % calls the drawing2gcode function on the drawing_input, then puts the
    % results in the text area.

    global drawing_input;
    if length(drawing_input{1}) > 0
        gcode = drawing2gcode(speed.Value);
        text_area.Value = gcode;
    end
    
end

function linebreakCallback(~,~,text_area)
    %This is the callback function for the line break button. It just
    %prints a ';' on the next line.
    
    index = length(text_area.Value)+1;
    text_area.Value{index} = sprintf(';');
    
end

function deleteLineCallback(~,~,text_area)
    %This is the callback function for the delete line button. It just
    %deletes the last line in the G-Code text box

    temp = {};
    if length(text_area.Value) > 1
        for i = 1:1:(length(text_area.Value)-1)
            temp{i} = text_area.Value{i};
        end
        text_area.Value = temp;
    end
    
end

function pxChange(obj,~,ef)
    % This is the callback for the P and X dropdown on the G04 argument
    % inputs. It updates the currentgcode value and also deletes the
    % editfield value if the drop down changes value
    
    global gui;
    global currentgcode;
    children = get(gui,'Children');
    
    for i = 1:1:length(children)
       if length(children(i).Type) == 11
            children(i).Value = '';
       end
    end
    currentgcode(2) = convertCharsToStrings([obj.Value ef.Value]);
    
end

function clearGcodeCallback(~,~,text_area)
    % This is the callback function for the clear gcode button. It just
    % sets the text_area to be equal ';'
    
    text_area.Value = sprintf(';');
    
end

function send2bufferCallback(~,~,text_area)
    % This function is the callback for the send to buffer button. It adds
    % the lines from the text area to the buffer, then deletes the text
    % area.

    global gcode_buffer;
    for i = 1:1:length(text_area.Value)
        gcode_buffer = [{text_area.Value{i}}; gcode_buffer];
    end
    text_area.Value = {''};
    update_clear_btn();
    update_bufferfig();
    
end

function directionBtnCallback(~,~,dir,dist,speed)
    % This is the callback function for the directional buttons. It takes a
    % direction, distance, and speed, and then ti moves at that speed for
    % that dist in that direction.
    
    global gcode_buffer;

   %If user didn't set distance or speed, give them some default values
   if isempty(dist.Value)
       dist.Value = '1';
   end 
   if isempty(speed.Value)
       speed.Value = '4.5';
   end
   
   % If direction = Y+,Y-,X+,X-,Z+,Z-
   %    add 'G91' to buffer (Set incremental movement)
   %    add 'G01' to buffer with distance and speed (Move in the specified
   %        direction
   %    add 'G90' to buffer (Set absolute movement)
   if dir == "Y+"
       gcode_buffer = [{'G91;'}; gcode_buffer ];
       gcode_buffer = [{['G01 Y'  dist.Value  ' F'  speed.Value  ';']}; gcode_buffer ];
       gcode_buffer = [{'G90;'}; gcode_buffer ];
   elseif dir == "X-"
       gcode_buffer = [{'G91;'}; gcode_buffer ];
       gcode_buffer = [{['G01 X-'  dist.Value  ' F'  speed.Value  ';']}; gcode_buffer ];
       gcode_buffer = [{'G90;'}; gcode_buffer ];
   elseif dir == "X+"
       gcode_buffer = [{'G91;'}; gcode_buffer ];
       gcode_buffer = [{['G01 X'  dist.Value  ' F'  speed.Value  ';']}; gcode_buffer ];
       gcode_buffer = [{'G90;'}; gcode_buffer ];
   elseif dir == "Y-"
       gcode_buffer = [{'G91;'}; gcode_buffer ];
       gcode_buffer = [{['G01 Y-'  dist.Value  ' F'  speed.Value  ';']}; gcode_buffer ];
       gcode_buffer = [{'G90;'}; gcode_buffer ];
   elseif dir == "Z+"
       gcode_buffer = [{'G91;'}; gcode_buffer ];
       gcode_buffer = [{['G01 Z'  dist.Value  ' F'  speed.Value  ';']}; gcode_buffer ];
       gcode_buffer = [{'G90;'}; gcode_buffer ];
   elseif dir == "Z-"
       gcode_buffer = [{'G91;'}; gcode_buffer ];
       gcode_buffer = [{['G01 Z-'  dist.Value  ' F'  speed.Value  ';']}; gcode_buffer ];
       gcode_buffer = [{'G90;'}; gcode_buffer ];
   elseif dir == "home"
       gcode_buffer = [{'G28;'}; gcode_buffer ];
   else
   
   end
   
   %Update the clear button because values have been added to the buffer
   update_clear_btn();
   update_bufferfig();
   
end

function bufferCallback(~,~)
    %This is the view gcode buffer button callback. It shows a figure which
    %contains a text area which contains the contents of the G-Code Buffer
    global bufferfig;
    global gcode_buffer;
    bufferfig = uitextarea('Position',[50 20 460, 380]);
    bufferfig.Parent.Name = 'G-Code Buffer';

    bufferfig.Value = flip(gcode_buffer);

end

function bufferClearCallback(~,~)
    %This function is the buffer clear button callback. It clears the
    %buffer and updates the number on itself.

    global gcode_buffer;
    gcode_buffer = [];
    update_clear_btn();
    update_bufferfig();

end

function sensorViewCallback(~,~)
    %This is the callback function for the button that opens the sensor
    %data view window. It opens the sensor data view window.

    global sensorViewFig;
    
    %Make the figure and the headings
    sensorViewFig = uifigure('Position',[200 100 600 500],'Tag','svf');
    axes_label = uilabel(sensorViewFig,'Text', 'Data Points', 'Position',[45 460 200 40],'FontSize',24);
    info_label = uilabel(sensorViewFig,'Text', 'Sensor Data', 'Position',[390 460 200 40],'FontSize',24);
    
    %Label and values for coordinates (data from Arduino)
    coord_label = uilabel(sensorViewFig,'Text', 'Coordinates (Inches):', 'Position',[390 430 200 25],'FontSize',16);
    coord_val = uilabel(sensorViewFig,'Text', ' ', 'Position',[390 410 200 25],'Tag','COORD');
    
    %Label and value for light sensor (data from Arduino)
    lsv_label = uilabel(sensorViewFig,'Text', 'Light Sensor Value:', 'Position',[390 360 200 25],'FontSize',16);
    lsv_val = uilabel(sensorViewFig,'Text', ' ', 'Position',[390 340 200 25],'Tag','LSV');
    
    %Label and value for RFID (data from Arduino)
    RFID_label = uilabel(sensorViewFig,'Text', 'RFID Value:', 'Position',[390 290 200 25],'FontSize',16);
    RFID_val = uilabel(sensorViewFig,'Text', ' ', 'Position',[390 270 200 25],'Tag','RFID');
    
    %Axes that will show data points
    dataAxes = uiaxes(sensorViewFig,'Position',  [20, 20, 340, 440],'ButtonDownFcn',@dataAxesCallback); %Create Axes
    dataAxes.XLim = [0 8.5]; %Set axes limits
    dataAxes.YLim = [0 11];
    dataAxes.PickableParts = 'all'; %Make axes clickable
    dataAxes.HitTest = 'on'; %Make axes clickable
    dataAxes.NextPlot = 'replacechildren'; 
    
    %Scroll buttons and data point number. Scrolling increases/decreases
    %data point number.
    scroll_left = uibutton(sensorViewFig,'push','Text','<<','Position',[390, 50, 90, 22],'ButtonPushedFcn',{@leftScrollCallback,dataAxes});
    scroll_right = uibutton(sensorViewFig,'push','Text','>>','Position',[490, 50, 90, 22],'ButtonPushedFcn',{@rightScrollCallback,dataAxes});
    dpn_val = uilabel(sensorViewFig,'Text', 'Data Point Number: ', 'Position',[390 75 200 25],'Tag','DPN');
    
    %Plot the sensor data on the axes
    update_sensor_data(dataAxes);
    
end

 function runCallback(obj,~)
    %This is the callback function for the run button. It starts running
    %whatever Gcode sequence is currently in the buffer by sending a start
    %signal to the Arduino. The Arduino responds with some initial location
    %and sensor data, then MATLAB knows that it can send the first gcode
    %command
 
    global gcode_buffer;
    global s;
    global serial_on;
    global prgm_end;
    
    %Program end set to 0 means the program is starting.
    prgm_end = 0;
    serial_on = 1;
    
    %If theres stuff in the buffer and run has been turned on, send the
    %start signal
    if length(gcode_buffer) > 0 && obj.Value == 1
        %serial_on = 0;
        t = timer('TimerFcn',@timer_trash,'StopFcn',{@send_gcode,s},'StartDelay',0.05);
        start(t);
    %Otherwise, turn the button back off
    else
        obj.Value = 0;
    end
    
    %If the button has been unpressed, press it again
    if obj.Value == 0
        obj.Value = 1;
    end
    
    %If there's nothing in the buffer, the program can be ended by
    %unpressing manually
    if length(gcode_buffer) == 0 && obj.Value == 1
        obj.Value = 0;
    end
    
 end

function leftScrollCallback(~,~,dataAxes)
    %This is the callback function for the left scroll button in the sensor
    %view window. This decreases the datapoint number so that you can look
    %at the next data point on the list
    
    global sensor_data;
    sds = size(sensor_data);
    
    %Find the data point colored blue
    last = findobj(dataAxes.Children,'Color',[0 0 1]);
    if size(last) > 0
        %get the data point number from the tags
        id = last.Tag;
        newid = str2num(id)-1; %decr data point number (go to next point)
        %If the data point number is 0, go to the highest one
        if newid == 0
           newid = sds(1);
        end
        %Update the sensor view with the new selected data point
        update_sensor_view(num2str(newid),dataAxes);
    else
        %If none had been selected previously, just select the first one
        update_sensor_view('1',dataAxes);
    end
end

function rightScrollCallback(~,~,dataAxes)
   %This is the callback function for the right scroll button in the sensor
    %view window. This increases the datapoint number so that you can look
    %at the next data point on the list
    
    global sensor_data;
    sds = size(sensor_data);
    
    %Find the data point colored blue
    last = findobj(dataAxes.Children,'Color',[0 0 1]);
    if size(last) > 0
        %Grab the data point number and increase it by one
        id = last.Tag;
        newid = str2num(id)+1;
        %If the new data point number is too large, go back to the first
        %one
        if newid == sds(1)+1
           newid = 1;
        end
        %Update the sensor view with the new selected data point
        update_sensor_view(num2str(newid),dataAxes);
    else
        %If no point had been selected previously, just select number 1
        update_sensor_view('1',dataAxes);
    end
end

function dataPointCallback(self,~,dataAxes)
    %This is the callback function for clicking on a data point. It simply
    %grabs the tag from itself and then passes it on to the
    %update_sensor_view function so that it can select this new point.
    
    id = self.Tag;

    update_sensor_view(id,dataAxes);
    
end

function dirSendCallback(~,~,dir_ser,ser_get)
    % Callback function for the send button in the direct serial comm
    % section.
    
    global s;
    
    % For every item in the serial send bar, send it on serial and delete
    % the line from the text area.
    for idx = 1:1:length(dir_ser.Value)
        disp(dir_ser.Value{idx});
        ser_get.Value{end+1} = '';
        ser_get.Value{end+1} = sprintf('Outgoing Serial');
        ser_get.Value{end+1} = sprintf(dir_ser.Value{idx});
        scroll(ser_get,'bottom');
        
        % Only send if its not an empty line.
        if ~isempty(dir_ser.Value{idx})
            writeline(s,dir_ser.Value{idx});
            dir_ser.Value{idx} = '';
        end
    end
end

function serGetCallback(self,~)
    %Scroll the serial monitor to the bottom
    
    scroll(self,'bottom');
end

function dataAxesCallback(~,~)
    %This callback function does nothing
end
% ======================== %


% === UPDATE FUNCTIONS === %
function update_clear_btn()
    % This function updates the display on the buffer clear button that
    % tells you how many lines are in the buffer

   global gui;
   global gcode_buffer;
   children = get(gui,'Children');
   % Find the button by Tag and then set the number in the text to include
   % new size of gcode_buffer
   for i = 1:1:length(children)
       if children(i).Tag == 'C'
            children(i).Text = ['Clear Buffer (' int2str(length(gcode_buffer)) ')'];
       end
   end
   
   if length(gcode_buffer) == 0
       run_btn = findobj(gui.Children,'Tag','RUN');
       run_btn.Value = 0;
   end
   
end

function update_bufferfig()
    %This function updates the contents of the text box in the external
    %figure which shows the GCode buffer.
    
    global bufferfig;
    global gcode_buffer;
    
    %If the gcode buffer has stuff in it and the figure exists, set the new
    %value of the text box
    if length(gcode_buffer) > 0 && sum(size(findobj('Value',bufferfig))) > 0
        bufferfig.Value = flip(gcode_buffer);
    end
    
end

function update_sensor_data(dataAxes)
    %This function updates the plot in the sensor view window. It plots all
    %of the newest sensor data on the axes.
    
    global sensor_data;
    
    %Set the axes to replace old plots with new ones, then plot a garbage
    %point
    dataAxes.NextPlot = 'replacechildren';
    plot(dataAxes,-1,-1,'ro');
    
    %Set axes to ADD all new plots
    dataAxes.NextPlot = 'add';
    
    sd = size(sensor_data);
    
    %Plot all of the points
    for i = 1:1:sd(1)
        plot(dataAxes,sensor_data(i,1),sensor_data(i,2),'ro','Tag',num2str(i),'ButtonDownFcn',{@dataPointCallback,dataAxes});
    end
        
end

function update_sensor_view(id,dataAxes)
    %This function updates the sensor view window given a newly selected
    %data point. The data point with data point number 'id' is turned blue
    %and its data is shown on the right hand side of the window.
    
    global sensorViewFig;
    global sensor_data;
    
    %Find the point that's colored blue
    last = findobj(dataAxes.Children,'Color',[0 0 1]);
    
    %If there was a blue point, turn it back to red
    if size(last) > 0
        last.Color = [1 0 0];
        last.MarkerSize = 6;
    end
    
    %Get the point with data point number id and turn it blue
    curr = findobj(dataAxes.Children,'Tag',id);
    curr.Color = [0 0 1];
    curr.MarkerSize = 10;
    
    nid = str2num(id);
    
    %Grab all of the data display labels
    coord_lbl = findobj(sensorViewFig.Children,'Tag','COORD');
    lsv_lbl = findobj(sensorViewFig.Children,'Tag','LSV');
    rfid_lbl = findobj(sensorViewFig.Children,'Tag','RFID');
    dpn_lbl = findobj(sensorViewFig.Children,'Tag','DPN');
    
    %get all of the values to display from the sensor data (depends on id)
    x = num2str(sensor_data(nid,1));
    y = num2str(sensor_data(nid,2));
    z = num2str(sensor_data(nid,3));
    lsv = num2str(sensor_data(nid,7));
    rfid = num2str(sensor_data(nid,8));
    
    %Change the text of the labels to match the data gathered from
    %sensor_data
    coord_lbl.Text = ['X: ' x '    Y: ' y '    Z: ' z];
    lsv_lbl.Text = lsv;
    rfid_lbl.Text = rfid;
    dpn_lbl.Text = ['Data Point Number: ' id];
    
end
% ======================== %


% === SERIAL FUNCTIONS === %
function readSerialData(src,~)
    %This is the callback function for the serial device. This is called
    %when there are bytes available.
    
    global sensor_data;
    global gui;
    global sensorViewFig;  
    global serial_on;
    values = zeros(1,8);
    
    %If there are bytes available and serial is enabled, grab all of the
    %new data.
    serial_on = 1;
    if src.NumBytesAvailable > 0 && serial_on == 1
        values = getSerial(src);
        sensor_data = [sensor_data; values];
        
        %If the sensor view window is open, update the axes.
        if size(findobj(sensorViewFig,'Type','figure')) > 0
            dataAxes = findobj(sensorViewFig.Children,'Type','axes');
            if size(dataAxes) > 0 
                update_sensor_data(dataAxes);
            end  
        end
    end

end

 function [values] = getSerial(src)
    %This function actually accesses the serial data and parses it into the
    %sensor data matrix
    
    global gcode_buffer;
    global gui;
 
    character = ' ';
    data = '';
    current = '';
    j = -1;
    lastj = -1;
    values = zeros(1,8);
    
    %Keep getting data until it hits a G (end of code) or a # (end of transmission);
    while character ~= 'G' && character ~='#'
        character = read(src,1,"char");
        data = [data  character];    
    end
    
    %Parse data and place into sensor_data depending on the letter
    %preceding it.
    for i = 1:1:length(data)
        if data(i) == 'X' %X location
            j = 1;
        elseif data(i) == 'Y' %Y location
            j = 2;      
        elseif data(i) == 'Z' %Z location
            j = 3;
        elseif data(i) == 'A' %X location
            j = 4;
        elseif data(i) == 'B' %Y location
            j = 5;      
        elseif data(i) == 'C' %Z location
            j = 6;
        elseif data(i) == 'L' %Light sensor value
            j = 7;
        elseif data(i) == 'R' %RFID value
            j = 8;
        elseif data(i) == 'G' %Send another Gcode command
            j = -1;
            %For some reason, the serial write command cannot be used
            %within the serial read callback, so short timers have to be
            %used.
            if ~isempty(gcode_buffer)
                t = timer('TimerFcn',@timer_trash,'StopFcn',{@send_gcode,src},'StartDelay',0.1);
                start(t);
            end
        elseif data(i) == '#'
            while character ~= 'G'
                character = read(src,1,"char");    
            end
            character = read(src,1,"char");
            j=-1;
        end
        
        %If theres a new letter, put all of the previous numbers into the
        %sensor values
        if j ~= lastj && lastj ~= -1
            values(1,lastj) = str2double(current(2:end));
            current = '';
        end
        
        %If j is unchanged (same letter), keep adding the information
        if j ~= -1
            current = [current data(i)];
        end
        
        lastj = j;
    end
    
    ser_get = findobj(gui,'Tag','SER');
    
    if sum(values(1:3)) ~= 0 
        disp("Incoming Location Data:");
        disp(values(1:3));
        ser_get.Value{end+1} = sprintf('Incoming Location Data:');
        ser_get.Value{end+1} = sprintf(['X: ' num2str(values(1)) '  Y: ' num2str(values(2)) '  Z: ' num2str(values(3))]);
        scroll(ser_get,'bottom');
    end
    
    if sum(values(4:6)) ~= 0
        disp("Incoming Thread Length Data:");
        disp(values(4:6));
        ser_get.Value{end+1} = sprintf('Incoming Thread Length Data:');
        ser_get.Value{end+1} = sprintf(['A: ' num2str(values(4)) '  B: '  num2str(values(5)) '  C: ' num2str(values(6))]);
        scroll(ser_get,'bottom');
    end
    
 end

 function send_gcode(~,~,src)
    %This function sends 1 line of gcode through serial to the Arduino.
    %Because of the nature of the serial read callback, this function has
    %to be set up as a callback function for a timer.
    
    global gcode_buffer;
    global serial_on;
    global gui;
    global prgm_end;
    
    % Grab the serial monitor
    ser_get = findobj(gui,'Tag','SER');
    
    serial_on = 0;
    
    %If the gcode buffer has stuff in it and theres nothing more to read,
    %send the stuff.
    if length(gcode_buffer) > 0 && src.NumBytesAvailable == 0
         %get rid of all of the empty lines or line breaks
         while (sum(gcode_buffer{end} == ';') == 1 && length(gcode_buffer{end}) == 1) || length(gcode_buffer{end}) == 0
             gcode_buffer(end) = [];
         end
         writeline(src,gcode_buffer{end});
         disp("Outgoing GCode:");
         disp(gcode_buffer{end});
         % Print gcode to serial monitor
         ser_get.Value{end+1} = '';
         ser_get.Value{end+1} = sprintf('Outgoing Gcode');
         ser_get.Value{end+1} = sprintf(gcode_buffer{end});
         scroll(ser_get,'bottom');
         gcode_buffer(end) = [];
         update_clear_btn();
    end
    
    %Only turn the serial back on if the program is not ended
    if prgm_end == 0
         serial_on = 1;
    end
    
    update_bufferfig();
    
 end
   
 
 % === PROGRAM FLOW FUNCTIONS === %
 function timer_trash(~,~)
    %Timers have to have a function to be executed when they start. Nothing
    %needed to happen at the start of the timers in this program, so this
    %function is empty.
 end
 
 function end_program(~,~)
    %This function is run when the current gcode sequence has run its
    %course. It deactivates serial, deactivates the run button, and sets
    %the program end flag to true.
    
    global gui;
    global serial_on;
    global prgm_end;
    
    run_btn = findobj(gui.Children,'Tag','RUN');
    run_btn.Value = 0;
    serial_on = 0;
    prgm_end = 1;
    
 end
 

 
