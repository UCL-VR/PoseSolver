function [] = plot_stream_timings(A)
%PLOT_STREAM_TIMINGS Plots box-plots summarising the update intervals of
%each stream (a stream being a device, marker and data type combination)

figure;
hold all;

ids = unique(A(:,8))';

for id = ids
    rows = A(:,8) == id;
    events = A(rows,4);
    intervals = diff(events);
    boxplot(intervals,'Positions',double(id));
end

XTicks = [];

for id = ids
    rows = A(:,8) == id;
    B = A(rows,:);
    row = B(1,:);
    [~,src,marker] = decode_stream(row);
    XLabels(id) = (src + " " + num2str(marker));
    XTicks(id) = id;
end

h = gca;
h.XTick = XTicks;
h.XTickLabel = XLabels;

ylim([-0.01,0.1]);

end

