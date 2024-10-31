classdef mobile_car < handle
    properties
        car_side;
        car_wheel_dis;
        car_radius;
        car_center;
        car_bias;
        car_corners;
        
        wheel_side; 
        wheel1_corners;
        wheel2_corners;
        wheel1_center;
        wheel2_center;
        
        base_car_corners;
        base_wheel1_corners;
        base_wheel2_corners;
        
        
        
        current_rotation_angle;
        rotation_angle_temp;
    end
  
    methods
        function obj=mobile_car(car_side,car_wheel_dis,wheel_side,car_bias)
            obj.car_side = car_side;
            obj.car_wheel_dis = car_wheel_dis;
            obj.wheel_side = wheel_side;
            obj.car_bias = car_bias;
            obj.car_radius = (car_side / 2) * sqrt(2);
            car_corners=[ obj.car_side/2  obj.car_side/2;
                         -obj.car_side/2  obj.car_side/2;
                         -obj.car_side/2 -obj.car_side/2;
                          obj.car_side/2 -obj.car_side/2;
                          obj.car_side/2  obj.car_side/2;];
            obj.car_corners = car_corners + car_bias;
            obj.base_car_corners = car_corners + car_bias;
            obj.car_center = [0 0];
            
            
            wheel2_center = [car_bias(1) - car_side/2 - car_wheel_dis - wheel_side/2 0];   
            wheel1_center = [0 car_bias(2) - car_side/2 - car_wheel_dis - wheel_side/2];   
            obj.wheel2_corners = [wheel2_center(1)+wheel_side/2 wheel2_center(2)+wheel_side/2;
                                  wheel2_center(1)-wheel_side/2 wheel2_center(2)+wheel_side/2;
                                  wheel2_center(1)-wheel_side/2 wheel2_center(2)-wheel_side/2;
                                  wheel2_center(1)+wheel_side/2 wheel2_center(2)-wheel_side/2;
                                  wheel2_center(1)+wheel_side/2 wheel2_center(2)+wheel_side/2;];
            obj.wheel1_corners = [wheel1_center(1)+wheel_side/2 wheel1_center(2)+wheel_side/2;
                                  wheel1_center(1)-wheel_side/2 wheel1_center(2)+wheel_side/2;
                                  wheel1_center(1)-wheel_side/2 wheel1_center(2)-wheel_side/2;
                                  wheel1_center(1)+wheel_side/2 wheel1_center(2)-wheel_side/2;
                                  wheel1_center(1)+wheel_side/2 wheel1_center(2)+wheel_side/2;];
%             obj.base_wheel1_corners = obj.wheel1_corners;
%             obj.base_wheel2_corners = obj.wheel2_corners;
            obj.current_rotation_angle = 0;
            obj.rotation_angle_temp = 0;
%             obj.wheel1_center = (obj.base_wheel1_corners(1,:) + obj.base_wheel1_corners(3,:)) / 2;
%             obj.wheel2_center = (obj.base_wheel2_corners(1,:) + obj.base_wheel2_corners(3,:)) / 2;
        end
        
        function handle = plot(obj)
            handle = plot(obj.car_corners(:,1),obj.car_corners(:,2),'r-','Linewidth',2);hold on
            plot(obj.car_center(1),obj.car_center(2),'k.','markersize',35,'Linewidth',2);hold on
            plot(obj.wheel1_corners(:,1),obj.wheel1_corners(:,2),'b-','Linewidth',2);hold on
            plot(obj.wheel2_corners(:,1),obj.wheel2_corners(:,2),'b-','Linewidth',2);hold on
            
            set(gca,'XLim',[-0.3 1.0] + 0)
            set(gca,'YLim',[-0.3 1.0] + 0)
            daspect([1 1 1])
        end
        
        function obj=control(obj,u)
            if u(1) ~= 0 && u(2) == 0
%                 obj.car_bias(1) = obj.car_bias(1) + u(1) * cos(obj.current_rotation_angle);
%                 obj.car_bias(2) = obj.car_bias(2) + u(1) * sin(obj.current_rotation_angle);
                obj.car_corners(:,1) = obj.car_corners(:,1) + u(1) * cos(obj.current_rotation_angle);
                obj.car_corners(:,2) = obj.car_corners(:,2) + u(1) * sin(obj.current_rotation_angle);
                
                obj.wheel1_corners = obj.wheel1_corners + u(1) * [cos(obj.current_rotation_angle) sin(obj.current_rotation_angle)];
                obj.wheel2_corners = obj.wheel2_corners + u(1) * [cos(obj.current_rotation_angle) sin(obj.current_rotation_angle)];
            elseif u(1) == 0 && u(2) ~= 0
                obj.car_corners(:,1) = obj.car_corners(:,1) + u(2) * cos(obj.current_rotation_angle);
                obj.car_corners(:,2) = obj.car_corners(:,2) + u(2) * sin(obj.current_rotation_angle);
                
                obj.wheel1_corners = obj.wheel1_corners + u(1) * [cos(obj.current_rotation_angle) sin(obj.current_rotation_angle)];
                obj.wheel2_corners = obj.wheel2_corners + u(1) * [cos(obj.current_rotation_angle) sin(obj.current_rotation_angle)];
                
%                 obj.car_bias(1) = obj.car_bias(1) + u(2) * cos(obj.current_rotation_angle);
%                 obj.car_bias(2) = obj.car_bias(2) + u(2) * sin(obj.current_rotation_angle);
%                 obj.car_corners = obj.car_corners + obj.car_bias;
%                 obj.wheel1_corner = obj.wheel1_corner + obj.car_bias;
%                 obj.wheel2_corner = obj.wheel2_corner + obj.car_bias;
            end
            
            
            if u(1) ~= 0 && u(2) ~= 0
                v = u(1) - u(2);
                w = v / obj.car_radius;
                
                obj.current_rotation_angle = w * 1 + obj.rotation_angle_temp;
                obj.rotation_angle_temp = obj.current_rotation_angle;
                
                center_ = (obj.car_corners(1,:)+obj.car_corners(3,:))/2;
                obj.car_corners = (obj.car_corners-center_) * obj.rotation(w) + center_;
                obj.wheel1_corners = (obj.wheel1_corners - center_) * obj.rotation(w) + center_;
                obj.wheel2_corners = (obj.wheel2_corners - center_) * obj.rotation(w) + center_;
                
%                 obj.car_corners = obj.base_car_corners * obj.rotation(obj.current_rotation_angle) + obj.car_center;
%                 obj.wheel1_corners = obj.base_wheel1_corners * obj.rotation(obj.current_rotation_angle) + obj.wheel1_center;
%                 obj.wheel2_corners = obj.base_wheel2_corners * obj.rotation(obj.current_rotation_angle) + obj.wheel2_center;
                
%                 obj.car_corners = obj.car_corners * obj.rotation(w);
%                 obj.wheel1_corner = obj.wheel1_corner * obj.rotation(w);
%                 obj.wheel2_corner = obj.wheel2_corner * obj.rotation(w);
            end
            
            obj.car_center = (obj.car_corners(1,:) + obj.car_corners(3,:)) / 2;
%             obj.wheel1_center = (obj.wheel1_corners(1,:) + obj.wheel1_corners(3,:)) / 2;
%             obj.wheel2_center = (obj.wheel2_corners(1,:) + obj.wheel2_corners(3,:)) / 2;
        end   
        function T = rotation(obj,alpha)
            T = [cos(alpha) sin(alpha);
                -sin(alpha) cos(alpha)];
        end
    end
end