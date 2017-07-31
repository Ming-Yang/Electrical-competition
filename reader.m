clear all;
close all;
file = input('Please input the sequence of text file:');
source = importdata(['.\Run data\data2\' num2str(file) '.txt']);

str = deblank(source.textdata);
split_str = regexp(str, '\t', 'split');
column = [9 11 12 13];%修改第一输出图像对应列数
figure;
plot(source.data(:,1),source.data(:,column(1)),'r',...
    source.data(:,1),source.data(:,column(2)),'g',...
    source.data(:,1),source.data(:,column(3)),'b',...
    source.data(:,1),source.data(:,column(4)),'k',...
    'linewidth',2);
xlabel(split_str{1}{1});
ylabel([split_str{1}{column(1)} ',' ...
    split_str{1}{column(2)} ',' ...
    split_str{1}{column(3)} ',']);
legend(split_str{1}{column(1)},split_str{1}{column(2)},split_str{1}{column(3)});

mode = input('Please input the number of forward step:');
switch mode
    case 0
        return;
    case 1
        column = [5 6 7];
        figure;
        plot(source.data(:,1),source.data(:,column(1)),'r',...
            source.data(:,1),source.data(:,column(2)),'g',...
            source.data(:,1),source.data(:,column(3)),'b',...
            'linewidth',2);
        xlabel(split_str{1}{1});
        ylabel([split_str{1}{column(1)} ',' ...
            split_str{1}{column(2)} ',' ...
            split_str{1}{column(3)} ',']);
        legend(split_str{1}{column(1)},split_str{1}{column(2)},split_str{1}{column(3)});
    otherwise
        return
end
clear all;