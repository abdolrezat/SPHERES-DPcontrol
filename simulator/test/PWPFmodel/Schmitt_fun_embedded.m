function y = Schmitt_fun_embedded(u,Uout,Uon,Uoff)
persistent us

if isempty(us) 
    us = u;
end

if( abs(u) < Uoff )
    us = 0;
end

if( abs(u) > Uon )
    if(u > Uon)
        us = Uout;
    else
        us = -Uout;
    end
end

y = us;