function summary = replay_log_template(csv_file)
% Skeleton deterministic event replay. Adapt payload parsing to log_schema.md.
T=readtable(csv_file); T=sortrows(T,{'t_receive_us','sequence'});
c=eskf_default_config(); P0=eye(15);
s=eskf_init(zeros(3,1),zeros(3,1),[1;0;0;0],zeros(3,1),zeros(3,1),P0,c);
last_imu_us=nan;
for k=1:height(T)
    switch upper(string(T.sensor_id(k)))
        case "IMU"
            if ~isnan(last_imu_us)
                dt=double(T.t_measure_us(k)-last_imu_us)*1e-6;
                s=eskf_predict(s,[T.ax(k);T.ay(k);T.az(k)], ...
                    [T.gx(k);T.gy(k);T.gz(k)],dt);
            end
            last_imu_us=T.t_measure_us(k);
        case "BARO"
            [s,~]=eskf_update_baro(s,T.altitude(k),T.variance(k));
        otherwise
            % Add GPS/MAG payload mapping for the concrete logger.
    end
end
summary.state=s; summary.events=height(T);
end
