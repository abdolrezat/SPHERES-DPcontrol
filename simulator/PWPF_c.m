classdef PWPF_c < handle
    %PWPF_c Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        schmitt
        h
        Km
        Tm
        H_feed
        ufeedback
        u_accum
    end
    
    methods
        function this = PWPF_c(Km,Tm,h,Uout,Uon,Uoff,H_feed)
            % initialization
            this.ufeedback = 0;
            this.u_accum = 0;
            this.Km = Km;
            this.Tm = Tm;
            this.h = h;
            this.schmitt = Schmitt_trigger_c(Uout,Uon,Uoff);
            this.H_feed = H_feed;
        end
        
        function out = signal_update(obj, Uin)
            %calculate error
            e = Uin - obj.ufeedback*obj.H_feed;
            
            %Transfer Function 1/ first order delay
            udot = (obj.Km*e - obj.u_accum)/obj.Tm;
            obj.u_accum = obj.u_accum + udot*obj.h;
            % schmitt trigger
            out = obj.schmitt.signal_update(obj.u_accum);
            
            % feedback loop
            obj.ufeedback = out;
        end
        
    end
    
end

