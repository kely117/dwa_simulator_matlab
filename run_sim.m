
clear;
nos = ["hundred",'two_hundred','four_hundred'];

for ni=1:length(nos)
    no = nos(ni);
    waypoints = load(sprintf("field_4/%s_stops.txt",no));
    spots = load(sprintf("field_4/%s_spots.txt",no));
    robot= load(sprintf("field_4/%s_robot.txt",no));
    model = [1 115/2];
    results = simple_simulator(waypoints,robot,model);
    spot_to_spot_distance=sum(results.distance(2:length(results.distance)-1));
    spot_to_spot_time=sum(results.time(2:length(results.distance)-1));
    figure;
    hold on;
    scatter(spots(:,1),spots(:,2),"*r");
    scatter(waypoints(:,1),waypoints(:,2),"ob");
    scatter(robot(:,1),robot(:,2),"^b");
    plot(results.waypoints(:,1),results.waypoints(:,2),"-b");
    text(5,15,sprintf("distance travelled: %.2f m",spot_to_spot_distance));
    text(5,10,sprintf("time taken: %.2f minutes",spot_to_spot_time/60));
    title(sprintf("Spot spraying trajectory; field 4; %s spots.",strrep(no,'_',' ')));
    lgd = legend(["Spots","Stops","Start","Trajectory"],'FontSize',7);
    lgd.NumColumns = 2;
    legend('boxoff');
    hold off;
end 