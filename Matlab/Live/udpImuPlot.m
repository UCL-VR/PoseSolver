
% Udp data logging
% Based on https://uk.mathworks.com/matlabcentral/fileexchange/58379-udp-real-time-plotting-animated-lines

function udpImuPlot
    %This function sets up the graphs
    %And initializes the udp listening
    
    %Initialize variables
    global xlimit;
    global xcounter;
    global port;
    global x;
    global y;
    global z;
    global type;
  
    xlimit = 5000;
    xcounter = 0;
    port = 24693; 
    type = 2; % gyro
    range = 50;
    
    u = udpport("datagram","IPV4","LocalPort",port,"EnablePortSharing",true);
        
    %Initialize Plot Window
    uFigure = figure('NumberTitle','off',...
        'Name','Live Data Stream Plot',...
        'Color',[0 0 0],...
        'CloseRequestFcn',{@localCloseFigure,u});
    
    uAxes = axes('Parent',uFigure,...
        'YGrid','on',...
        'YColor',[0.9725 0.9725 0.9725],...
        'XGrid','on',...
        'XColor',[0.9725 0.9725 0.9725],...
        'Color',[0 0 0]);
    
    xlabel(uAxes,'Number of Samples');
    xlim([0 xlimit]);
    ylabel(uAxes,'Value');
    ylim([-1 1] * range);
    hold on; %Hold on to allow addition of multiple plots
    
    %Add more plots here to window if necessary
    x = animatedline('Color','r', 'MaximumNumPoints', xlimit);
    y = animatedline('Color','g', 'MaximumNumPoints', xlimit);
    z = animatedline('Color','b', 'MaximumNumPoints', xlimit);
    
    %Setup Udp object    
    configureCallback(u,"datagram",1,@onDatagramAvailable);
 end

function onDatagramAvailable(u,~)
    global xcounter;
    global x;
    global y;
    global z;
    global xlimit;
    global type;
    
    available = u.NumDatagramsAvailable;
    
    data = read(u,available,"single");
    data = reshape([data.Data],6,[])';
    
    % Filter
    data = data( data(:,1)==type, :);
    
    len = height(data);
    
    xd = (1:len) + xcounter;
    xcounter = xcounter + len;
   
    addpoints(x, xd, data(:,4));
    addpoints(y, xd, data(:,5));
    addpoints(z, xd, data(:,6));
        
    drawnow;
    
    if(xcounter > xlimit)
        clearpoints(x);
        clearpoints(y);
        clearpoints(z);
        xcounter = 0;
    end
end

function localCloseFigure(figureHandle,~,u)
    u.delete();
    clear u;
    delete(figureHandle)
    clear global counter;
end





