function [closest_pt, distance ] = ClosestPointOnEdgeToPoint( edges, pt) 
    
    if  dot( edges(:,1) - pt,  edges(:,2) - edges(:,1) ) > 0
        closest_pt = edges(:,1);
        distance = sqrt( sum((pt - closest_pt) .^ 2));
    elseif dot( edges(:,2) - pt, edges(:,1) - edges(:,2) ) > 0
        closest_pt = edges(:,2);
        distance = sqrt( sum((pt - closest_pt) .^ 2));
    else
        closest_pt = find_perpendicular(pt, edges(:,1), edges(:,2) );
        distance = sqrt( sum((pt - closest_pt) .^ 2));
        
end
