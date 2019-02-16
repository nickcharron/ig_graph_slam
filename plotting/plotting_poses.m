% This script is for plotting the poses outputted by ig_graph_slam
clc; clear; close all; format short;

files = {'opt_traj3.txt','opt_traj4.txt'};

for k = 1:size(files,2)

    fileID = fopen(files{k},'r');
    formatSpec = '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f';
    dataIn = textscan(fileID,formatSpec,'Delimiter',',');
    fclose(fileID);
    N = size(dataIn{1},1);

    t = zeros(1,N);
    X = zeros(4,4,N);
    tStart = dataIn{1}(1);
    axisDim = 1;

    figure(k)
    for i = 1:N
        t(1,i) = dataIn{1}(i) - tStart;
        X(:,:,i) = [dataIn{2}(i) , dataIn{3}(i), dataIn{4}(i), dataIn{5}(i);
                    dataIn{6}(i) , dataIn{7}(i), dataIn{8}(i), dataIn{9}(i);
                    dataIn{10}(i) , dataIn{11}(i), dataIn{12}(i), dataIn{13}(i);
                    dataIn{14}(i) , dataIn{15}(i), dataIn{16}(i), dataIn{17}(i)];
        drawAxis(X(:,:,i), axisDim)            
    end

end