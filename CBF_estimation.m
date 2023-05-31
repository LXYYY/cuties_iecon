function cbfparam = CBF_estimation(ptCloudOut,T,params)

% initial output
cbfparam = []; num = 0;

% remove ground
% indices = findPointsInROI(ptCloud,roi);
% ptCloudB = select(ptCloud,indices);
% transpose
% ptCloudtrans = pctransform(removeInvalidPoints(PtCloud),rigidtform3d(T));
% ptCloudOut = pctransform(removeInvalidPoints(PtCloud),rigidtform3d(T));
% 
% % remove ground
% [~,groundIdx,nongroundIdx] = pcfitplane(ptCloudtrans,0.3,[0 0 1],10);
% % groundPtsIdx = segmentGroundFromLidarData(PtCloud,ElevationAngleDelta=5);
% groundPtCloud = select(ptCloudtrans,groundIdx);
% % groundPtCloud.Count
% ptCloudOut = select(ptCloudtrans,nongroundIdx);
% % nonGroundPtCloud = select(PtCloud,~groundPtsIdx,'OutputSize','full');

% Segmentation and Clustering
labels = zeros(ptCloudOut.Count,1); numClusters = 0;
[labels,numClusters] = pcsegdist(ptCloudOut,params.d_cluster);
%
if numClusters ~=0
    % FOR each cluster find out plan set
    if num ==0
        cbfparam = [];
    end
    for i = 1:numClusters
        %define points
        index = find(labels == i);
        X = ptCloudOut.Location(index,:);
        if size(index,1) < 10
            continue
        end
        %fit plane and cuboid
        num = num +1;
        [~,inlierIndices,outlierIndices] = pcfitplane(pointCloud(X),params.d_plane);
        model = pcfitcuboid(pointCloud(X),inlierIndices);
        % update param
        xc = model.Center(1);yc = model.Center(2);zc = model.Center(3);
        a = model.Dimensions(1)/2 +1e-5;
        b = model.Dimensions(2)/2 +1e-5;
        c = model.Dimensions(3)/2 +1e-5;
        R = eul2rotm(-model.Orientation*pi/180,'XYZ');
        cbfparam = [cbfparam;[xc yc zc a b c -model.Orientation*pi/180 0]];
        % while
        while(size(outlierIndices,1) > 10)
            num = num+1;
            X = X(outlierIndices,:);
            % ptCloudOut = select(ptCloudOut,outlierIndices);
            [~,inlierIndices,outlierIndices] = pcfitplane(pointCloud(X),params.d_plane);
            model=pcfitcuboid(pointCloud(X(inlierIndices,:)));
            xc = model.Center(1);yc = model.Center(2);zc = model.Center(3);
            a = model.Dimensions(1)/2 +1e-5;
            b = model.Dimensions(2)/2 +1e-5;
            c = model.Dimensions(3)/2 +1e-5;
            R = eul2rotm(-model.Orientation*pi/180,'XYZ');
            cbfparam = [cbfparam;[xc yc zc a b c -model.Orientation*pi/180 0]];
        end
    end
else
    cbfparam = [];
end
end