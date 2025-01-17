global phaseA;
global phaseB;
global phaseC;
global phaseD;
global queue;
global period;
global min_green;
global max_green;
global sim_time;
global sim_time_set;
global cur_phase;
global period_green;
global cars_arri;
global greenlist;
global redlist;
global control_list;
phaseA = 1;
phaseB = 2;
phaseC = 3;
phaseD = 4;
queue = ones(8,1);
period = 4;
min_green = 8;
max_green = 40;
sim_time = 0;
sim_time_set = 3600;
cur_phase = phaseA;
cur_phase_green = 0;
period_green = 0;
avg_green = 40;
round_count = 1;
round_time = 0;
round_car_left = 0;
total_wait_time = 0;
avr_wait_time = 0;
cars_left_total = 0;
greenlist = green();
redlist = red();
control_list = control(greenlist,redlist);
cars_arri = car_reach();

cars_arri = zeros(8,900);
for k = 1:12
    for i = 1:8
        for j = (((k-1)*75)+1):75*k
            if rate(k,i) <= 0.4
                cars_arri(i,j) = poissrnd(4*rate(k,i));
            else
                cars_arri(i,j) = binornd(5,rate(k,i));
            end
        end
    end
end

TRgreen = readfis('GreenLntensity');
greenlist = zeros(21,11);
for i = 1:1:21;
    for j = 1:11;
        rate = (j-1)/10;
        queue = i-1;
        greenlist(i,j) = evalfis([queue,rate],TRgreen);
    end
end

while sim_time < (sim_time_set-8)
    cur_phase_green = 0;
    sim_time = sim_time + min_green;
    cur_phase_green = cur_phase_green + min_green;
    queue = queue + cars_arri(:,sim_time/4 - 1) + cars_arri(:,sim_time/4);
    for i_pos = 1:8
        if queue(i_pos,1) > 20
            queue(i_pos,1) = 20;
        end
    end
    car_left = zeros(8,1);
    if cur_phase_green  == min_green
        car_left(cur_phase,1) = min(min_green,queue(cur_phase,1));
        car_left(cur_phase+4,1) = min(min_green,queue(cur_phase+4,1));
    else
        car_left(cur_phase,1) = min(period,queue(cur_phase,1));
        car_left(cur_ohase+4,1) = min(period,queue(cur_phase+4,1));
    end
    cars_left_total = cars_left_total + sum(car_left);
    round_car_left = round_car_left + sum(car_left);
    queue = queue - car_left;
    total_wait_time = queue*4 + total_wait_time;
    while (cur_phase_green < max_green) && (sim_time < sim_time_set) && (control_list(max(queue(queue(cur_phase,1),queue(cur_phase_4,1))+1,round(10*rate(floor(sim_time/300)+1,((queue(cur_phase) >= queue(cur_phase+4)) * cur_phase + (queue(cur_phase) < queue(cur_phase + 4)) * (cur_phase + 4)))) + 1,max(queue(((cur_phase == 4) * phaseA + (cur_phase ~= 4) * (cur_phase + 1)),1),queue(((cur_phase ==4 ) * (phaseA + 4)+(cur_phase ~= 4) * (cur_phase + 5)),1)) + 1) < 0.5)
        sim_time = sim_time + period;
        cur_phase_green = cur_phase_green + period;
        queue = queue + cars_arri(:,sim_time/4);
        for i_phase = 1:8
            if queue(i_phase,1) > 20
                queue(i_phase,1) = 20;
            end
        end
        car_left = zeros(8,1);
        if cur_phase_green == min_green
            car_left(cur_phase,1) = min(min_green,queue(cur_phase,1));
            car_left(cur_phase+4,1) = min(min_green,queue(cur_phase+4,1));
        else
            car_left(cur_phase,1) = min(period,queue(cur_phase,1));
            car_left(cur_phase+4,1) = min(period,queue(cur_phase+4,1));
        end
        queue = queue - car_left;
        total_wait_time = queue * 4 + total_wait_time;
        car_left_total = cars_left_total + sum(car_left);
        round_car_left = round_car_left + sum(car_left);
    end
    if cur_phase == phaseD
        cur_phase = phaseA;
        avr_wait_time = sum(total_wait_time) / round_car_left;
        disp(avr_wait_time)
        round_count = round_count + 1;
        round_car_left = 0;
        total_wait_time = 0;
    else
        cur_phase = cur_phase + 1;
    end
end

while sim_time < sim_time_set
    cur_phase_green = 0;
    sim_time = sim_time + max_green;

    while (cur_phase_green < max_green) && (sim_time < sim_time_set)
        cur_phase_green = cur_phase_green + period;
        queue = queue + cars_arri(:,sim_time/4);
        for i_phase = 1:8
            if queue(i_phase,1) > 20
                queue(i_phase,1) = 20;
            end
        end
        car_left = zeros(8,1);

            car_left(cur_phase,1) = min(period,queue(cur_phase,1));
            car_left(cur_phase+4,1) = min(period,queue(cur_phase+4,1));

        queue = queue - car_left;
        total_wait_time = queue * 4 + total_wait_time;
        cars_left_total = cars_left_total + sum(car_left);
        round_car_left = round_car_left + sum(car_left);
    end

        if cur_phase == phaseD
            cur_phase = phaseA;
            avr_wait_time = sum(total_wait_time) / round_car_left;
            disp(avr_wait_time)
            round_count = round_count + 1;
            round_car_left = 0;
            total_wait_time = 0;
        else
            cur_phase = cur_phase + 1;
        end
end
