load ../timing_frontalface.txt 
%load ../timing_profileface.txt 
load ../timing_legtracker.txt 
load ../timing_filter.txt 


size_frontalface = size(timing_frontalface)(1)
%size_profileface =  size(timing_profileface)(1)
size_legtracker = size(timing_legtracker)(1)
size_filter = size(timing_filter)(1)


time_frontalface = [1:size_frontalface]'./size_frontalface;
%time_profileface = [1:size_profileface]'./size_profileface;
time_legtracker = [1:size_legtracker]'./size_legtracker;
time_filter = [1:size_filter]'./size_filter;

figure; hold on;
plot(time_frontalface, timing_frontalface, 'b');
%plot(time_profileface, timing_profileface);
plot(time_legtracker, timing_legtracker,'r');
plot(time_filter, timing_filter,'g');
hold off;
