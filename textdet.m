image = imread('/home/tejaswikasarla/Pictures/testimage.jpg');
I = rgb2gray(image);

% Detect MSER regions.
[mserRegions, mserComponent] = detectMSERFeatures(I, ...
    'RegionAreaRange',[20 800],'ThresholdDelta',5);

% Use regionprops to measure MSER properties
mserStats = regionprops(mserComponent, 'BoundingBox', 'Eccentricity', ...
    'Solidity', 'Extent', 'Euler', 'Image');

% Compute the aspect ratio using bounding box data.
bboxes = vertcat(mserStats.BoundingBox);
w = bboxes(:,3);
h = bboxes(:,4);
aspectRatio = w./h;

% Threshold the data to determine which regions to remove. These thresholds
% may need to be tuned for other images.
threshold = aspectRatio' > 3;
threshold = threshold | [mserStats.Eccentricity] > .995 ;
threshold = threshold | [mserStats.Solidity] < .3;
threshold = threshold | [mserStats.Extent] < 0.2 | [mserStats.Extent] > 0.9;
threshold = threshold | [mserStats.EulerNumber] < -4;

% Remove regions which don't fall under threshold
mserStats(threshold) = [];
mserRegions(threshold) = [];

% Threshold the stroke width variation metric
strokeWidthThreshold = 0.4;

% Process the remaining regions
for j = 1:numel(mserStats)

    regionImage = mserStats(j).Image;
    regionImage = padarray(regionImage, [1 1], 0);

    distanceImage = bwdist(~regionImage);
    strokeImage = bwmorph(regionImage, 'thin', inf);

    strokeWidthValues = distanceImage(strokeImage);

    strokeWidthMetric = std(strokeWidthValues)/mean(strokeWidthValues);

    strokeWidthFilterIdx(j) = strokeWidthMetric > strokeWidthThreshold;

end

% Remove regions based on the stroke width variation
mserRegions(strokeWidthFilterIdx) = [];
mserStats(strokeWidthFilterIdx) = [];

% Get bounding boxes for all the regions
bboxes = vertcat(mserStats.BoundingBox);

% Convert from the [x y width height] bounding box format to the [xmin ymin
% xmax ymax] format for convenience.
xmin = bboxes(:,1);
ymin = bboxes(:,2);
xmax = xmin + bboxes(:,3) - 1;
ymax = ymin + bboxes(:,4) - 1;

% Expand the bounding boxes by a small amount.
expansionAmount = 0.02;
xmin = (1-expansionAmount) * xmin;
ymin = (1-expansionAmount) * ymin;
xmax = (1+expansionAmount) * xmax;
ymax = (1+expansionAmount) * ymax;

% Clip the bounding boxes to be within the image bounds
xmin = max(xmin, 1);
ymin = max(ymin, 1);
xmax = min(xmax, size(I,2));
ymax = min(ymax, size(I,1));

% Show the expanded bounding boxes
expandedBBoxes = [xmin ymin xmax-xmin+1 ymax-ymin+1];

% Create the graph and find the connected text regions within the graph
overlapRatio = bboxOverlapRatio(expandedBBoxes, expandedBBoxes);
g = graph(overlapRatio);

componentIndices = conncomp(g);

% Merge the boxes based on the minimum and maximum dimensions.
xmin = accumarray(componentIndices', xmin, [], @min);
ymin = accumarray(componentIndices', ymin, [], @min);
xmax = accumarray(componentIndices', xmax, [], @max);
ymax = accumarray(componentIndices', ymax, [], @max);

% Compose the merged bounding boxes using the [x y width height] format.
mergedBBoxes = [xmin ymin xmax-xmin+1 ymax-ymin+1];

% Remove bounding boxes that only contain one text region
numRegionsInGroup = histcounts(componentIndices);
mergedBBoxes(numRegionsInGroup == 1, :) = [];

% Show the final text detection result.
finalImage = insertShape(image, 'Rectangle', mergedBBoxes,'LineWidth',3);


figure
imshow(finalImage)
title('Detected Text')

ocrtxt = ocr(I, mergedBBoxes);
[ocrtxt.Text]