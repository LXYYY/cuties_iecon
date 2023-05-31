function [ranges, angles] = pointCloudToPolar(pointCloud)
    % Extract the X and Y coordinates
    X = pointCloud.Location(:, 1);
    Y = pointCloud.Location(:, 2);

    % Calculate the ranges and angles
    ranges = sqrt(X.^2 + Y.^2);
    angles = atan2(Y, X);
end
