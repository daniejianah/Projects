% Danie Jianah SONIZARA
% Master in Computer Vision Student 2017-2018
% University of Burgundy 
%%----------------------------------------------

function varargout = codeVP(varargin)
% CODEVP MATLAB code for codeVP.fig
%      CODEVP, by itself, creates a new CODEVP or raises the existing
%      singleton*.
%
%      H = CODEVP returns the handle to a new CODEVP or the handle to
%      the existing singleton*.
%
%      CODEVP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CODEVP.M with the given input arguments.
%
%      CODEVP('Property','Value',...) creates a new CODEVP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before codeVP_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to codeVP_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help codeVP

% Last Modified by GUIDE v2.5 07-May-2018 21:33:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @codeVP_OpeningFcn, ...
                   'gui_OutputFcn',  @codeVP_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before codeVP is made visible.
function codeVP_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to codeVP (see VARARGIN)

% Choose default command line output for codeVP
handles.output = hObject;
%Create tab group
handles.tgroup = uitabgroup('Parent', handles.figure1,'TabLocation', 'left');
handles.Tab1 = uitab('Parent', handles.tgroup, 'Title', 'Image');
handles.Tab2 = uitab('Parent', handles.tgroup, 'Title', 'Camera');
handles.Tab3 = uitab('Parent', handles.tgroup, 'Title', 'Video');
%Place panels into each tab
set(handles.panel1,'Parent',handles.Tab1)
set(handles.panel4,'Parent',handles.Tab2)
set(handles.panel7,'Parent',handles.Tab3)
%Reposition each panel to same location as panel 1
set(handles.panel4,'position',get(handles.panel1,'position'));
set(handles.panel7,'position',get(handles.panel1,'position'));
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes codeVP wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = codeVP_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in ImageButton.
function ImageButton_Callback(hObject, eventdata, handles)
% hObject    handle to ImageButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.panel1,'Visible','on')
set(handles.panel4,'Visible','off')
set(handles.panel7,'Visible','off')

% --- Executes on button press in Camerabutton.
function Camerabutton_Callback(hObject, eventdata, handles)
% hObject    handle to Camerabutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.panel4,'Visible','on')
set(handles.panel1,'Visible','off')
set(handles.panel7,'Visible','off')


% --- Executes on button press in Videobutton.
function Videobutton_Callback(hObject, eventdata, handles)
% hObject    handle to Videobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.panel7,'Visible','on')
set(handles.panel1,'Visible','off')
set(handles.panel4,'Visible','off')


% --- Executes on button press in LoadImage.
function LoadImage_Callback(hObject, eventdata, handles)
% hObject    handle to LoadImage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Here we load an image from any folder in the Desktop
global im im2
[path,user_cance] =imgetfile();
if user_cance
    msgbox(sprintf('Error'),'Error','Error');
    return
end
im = imread (path);
im=im2double(im); % convert to double
im2 =im ; %for backup process
axes(handles.axes1);
imshow(im);



% --- Executes on button press in SaveImage.
function SaveImage_Callback(hObject, eventdata, handles)
% hObject    handle to SaveImage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Here we save the output image 
[FileName,PathName,FilterIndex] = uiputfile({'.jpg;.jpeg;*.png','Images (.jpg,.jpeg,*.png)'},'');
if length(PathName) > 1 && length(FileName) > 1
    path = strcat(PathName ,FileName);
    img = getimage(handles.axes2);
    if isempty(img)
        handles.txt.String = 'Nothing to save yet';
    else
        imwrite(img,path)
        handles.txt.String = strcat('Result saved at ',path);
    end
end
        
name=fullfile(PathName,FileName);
imwrite((handles.output),name)


% --- Executes on selection change in ColorSpace.
function ColorSpace_Callback(hObject, eventdata, handles)
% hObject    handle to ColorSpace (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns ColorSpace contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ColorSpace
%% Here we change the colorspace of the chosen image 
global im im_colored
contents = get (hObject,'Value');
switch contents 
    case 1
        im_colored = im; %original image
        axes(handles.axes2);
        imshow(im_colored);
    case 2
        im_colored = rgb2gray(im);
        axes(handles.axes2);
        imshow(im_colored); 
        %converts the truecolor image RGB to the grayscale intensity image I
    case 3 
        im_colored = rgb2ycbcr(im);
        axes(handles.axes2);
        imshow(im_colored);
        %converts the RGB color space values in rgbmap to the YCbCr color space
    
    otherwise
        im_colored = rgb2gray(im);
        axes(handles.axes2);
        imshow(im_colored);
end


% --- Executes during object creation, after setting all properties.
function ColorSpace_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ColorSpace (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on selection change in ShapeDescriptors.
function ShapeDescriptors_Callback(hObject, eventdata, handles)
% hObject    handle to ShapeDescriptors (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns ShapeDescriptors contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ShapeDescriptors
%% Here we apply components shape descriptors
global im_colored
contents = get (hObject,'Value');

switch contents 
    case 1
        s = regionprops(im_colored,'BoundingBox');
        axes(handles.axes2);
        imshow(im_colored);hold on ;
        imshow(s);
        
    case 2
        
    otherwise 
end 
        


% --- Executes during object creation, after setting all properties.
function ShapeDescriptors_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ShapeDescriptors (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in cornerExtraction.
function cornerExtraction_Callback(hObject, eventdata, handles)
% hObject    handle to cornerExtraction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns cornerExtraction contents as cell array
%        contents{get(hObject,'Value')} returns selected item from cornerExtraction
%% Here we extract corners using Harris, FAST  
global im_colored
contents = get (hObject,'Value');

switch contents 
    case 1
        %% Corner Extraction using Harris
        %%applying sobel edge detector in the horizontal direction
        fx = [-1 0 1;-1 0 1;-1 0 1];
        Ix = filter2(fx,im_colored);
        % applying sobel edge detector in the vertical direction
        fy = [1 1 1;0 0 0;-1 -1 -1];
        Iy = filter2(fy,im_colored); 

        Ix2 = Ix.^2;
        Iy2 = Iy.^2;
        Ixy = Ix.*Iy;
        clear Ix;
        clear Iy;

        %applying gaussian filter on the computed value
        h= fspecial('gaussian',[7 7],2); 
        Ix2 = filter2(h,Ix2);
        Iy2 = filter2(h,Iy2);
        Ixy = filter2(h,Ixy);
        height = size(im_colored,1);
        width = size(im_colored,2);
        result = zeros(height,width); 
        R = zeros(height,width);

        Rmax = 0; 
        for i = 1:height
            for j = 1:width
                M = [Ix2(i,j) Ixy(i,j);Ixy(i,j) Iy2(i,j)]; 
                R(i,j) = det(M)-0.01*(trace(M))^2;
                if R(i,j) > Rmax
                    Rmax = R(i,j);
                end;
            end;
        end;
        cnt = 0;
        for i = 2:height-1
            for j = 2:width-1
                if R(i,j) > 0.1*Rmax && R(i,j) > R(i-1,j-1) && R(i,j) > R(i-1,j) && R(i,j) > R(i-1,j+1) && R(i,j) > R(i,j-1) && R(i,j) > R(i,j+1) && R(i,j) > R(i+1,j-1) && R(i,j) > R(i+1,j) && R(i,j) > R(i+1,j+1)
                    result(i,j) = 1;
                    cnt = cnt+1;
                end;
            end;
        end;
        [posc, posr] = find(result == 1);
        cnt ;
        axes(handles.axes2);
        imshow(im_colored);
        hold on;
        plot(posr,posc,'r.');
        
    case 2
        %% Corner Extraction using FAST algorithms
        corners = detectFASTFeatures(im_colored);
        axes(handles.axes2);
        imshow(im_colored); hold on;
        plot(corners.selectStrongest(50));

    otherwise 
end 



% --- Executes during object creation, after setting all properties.
function cornerExtraction_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cornerExtraction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in HoughTransform.
function HoughTransform_Callback(hObject, eventdata, handles)
% hObject    handle to HoughTransform (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns HoughTransform contents as cell array
%        contents{get(hObject,'Value')} returns selected item from HoughTransform
%% Here we use the hough transform to extract lines and circles 
global im_colored 
contents = get (hObject,'Value');

switch contents 
    case 1
        
        % Create a binary image
        BW = edge(im_colored,'canny');
        %Create the Hough transform using the binary image.
        [H,T,R] = hough(BW);
        %Find peaks in the Hough transform of the image.
        P  = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));
        %Find lines and plot them.
        lines = houghlines(BW,T,R,P,'FillGap',5,'MinLength',7);
        axes(handles.axes2);
        imshow(im_colored), hold on
        max_len = 0;
        for k = 1:length(lines)
            xy = [lines(k).point1; lines(k).point2];
            plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
        end
        

    case 2
        %Find all the circles with radius r pixels in the range [10, 100].
        [centers, radii, metric] = imfindcircles(im_colored,[10 100]);
        axes(handles.axes2);
        viscircles(centers, radii,'EdgeColor','b');
 
    
    otherwise 
end 

% --- Executes during object creation, after setting all properties.
function HoughTransform_CreateFcn(hObject, eventdata, handles)
% hObject    handle to HoughTransform (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in EdgeDetection.
function EdgeDetection_Callback(hObject, eventdata, handles)
% hObject    handle to EdgeDetection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns EdgeDetection contents as cell array
%        contents{get(hObject,'Value')} returns selected item from EdgeDetection
%% Here we apply edge detector operators
global im_colored 
contents = get (hObject,'Value');

switch contents 
    case 1
        sobelIm = edge(im_colored,'Sobel'); 
        axes(handles.axes2);
        imshow(sobelIm);
    case 2
        prewittIm = edge(im_colored,'prewitt');
        axes(handles.axes2);
        imshow(prewittIm); 
    case 3
        RobertsIm = edge(im_colored,'Roberts');
        axes(handles.axes2);
        imshow(RobertsIm); 
    case 4
        CannyIm = edge(im_colored,'Canny');
        axes(handles.axes2);
        imshow(CannyIm); 
    otherwise 
end 



% --- Executes during object creation, after setting all properties.
function EdgeDetection_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EdgeDetection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in Blur.
function Blur_Callback(hObject, eventdata, handles)
% hObject    handle to Blur (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns Blur contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Blur
%% Here we apply filters to blur the image
global im_colored
contents = get (hObject,'Value');

switch contents 
    case 1
        GaussFilter = imgaussfilt(im_colored,2); 
        axes(handles.axes2);
        imshow(GaussFilter);title('Gaussian filtered image, \sigma = 2')
    case 2
        h = fspecial('laplacian');
        LaplacianFilter = imfilter(im_colored,h);
        axes(handles.axes2);
        imshow(LaplacianFilter); 
    case 3
        h= fspecial('prewitt');
        PrewittFilter = imfilter(im_colored,h);
        axes(handles.axes2);
        imshow( PrewittFilter); 
    case 4
        h= fspecial('sobel');
        SobelFilter = imfilter(im_colored,h);
        axes(handles.axes2);
        imshow(SobelFilter); 
    otherwise 
end 

% --- Executes during object creation, after setting all properties.
function Blur_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Blur (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in MorphologicalOperators.
function MorphologicalOperators_Callback(hObject, eventdata, handles)
% hObject    handle to MorphologicalOperators (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns MorphologicalOperators contents as cell array
%        contents{get(hObject,'Value')} returns selected item from MorphologicalOperators
%% Here we apply apply morphological operators 
global im_colored se
contents = get (hObject,'Value');
switch contents 
    case 1
        %Create a flat, line-shaped structuring element se
        se = strel('line',10,80);
        dilateIm = imdilate(im_colored ,se);
        axes(handles.axes2);
        imshow(dilateIm);
    case 2
        %Create a nonflat ball-shaped structuring element.
        se = strel('line',10,80);
        erodeIm = imerode(im_colored ,se);
        axes(handles.axes2);
        imshow(erodeIm); 
    case 3
        %Create a disk-shaped structuring element with a radius of 5 pixels.
        se = strel('disk',4);
        openIm = imopen(im_colored ,se);
        axes(handles.axes2);
        imshow(openIm); 
     case 4
         %Create a disk-shaped structuring element. 
        %Use a disk structuring element to preserve the circular nature of the object. 
        %Specify a radius of 10 pixels so that the largest gap gets filled.
        se = strel('disk',4);
        closeIm =imclose(im_colored ,se) ;
        axes(handles.axes2);
        imshow(closeIm ); 
    otherwise 
end 


% --- Executes during object creation, after setting all properties.
function MorphologicalOperators_CreateFcn(hObject, eventdata, handles)
% hObject    handle to MorphologicalOperators (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in Histogram.
function Histogram_Callback(hObject, eventdata, handles)
% hObject    handle to Histogram (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns Histogram contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Histogram
%% Here we the histogram and compute
global im_colored im_hist im_hist_eq;
contents = get (hObject,'Value');

switch contents 
    case 1
        im_hist = im_colored; %calculates the histogram for the intensity image I and displays a plot of the histogram.
        axes(handles.axes2);
        imhist(im_hist);
    case 2
        im_hist_eq = histeq(im_colored);
        %Enhance contrast using histogram equalization;
        axes(handles.axes2);
        imshow(im_hist_eq); 
    case 3
        im_hist_eq = histeq(im_colored);
        %Enhance contrast using histogram equalization;
        axes(handles.axes2);
        imhist(im_hist_eq);
        
    otherwise 
end 


% --- Executes during object creation, after setting all properties.
function Histogram_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Histogram (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in AddNoise.
function AddNoise_Callback(hObject, eventdata, handles)
% hObject    handle to AddNoise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns AddNoise contents as cell array
%        contents{get(hObject,'Value')} returns selected item from AddNoise
%% Here we add noise to the original image
global  im_colored
contents = get (hObject,'Value');
switch contents 
    case 1
        im_noise = imnoise(im_colored,'gaussian');
        axes(handles.axes2);
        imshow(im_noise);
        % adds zero-mean, Gaussian white noise with variance of 0.01 to grayscale image
        
    case 2 
        im_noise = imnoise(im_colored,'poisson');
        axes(handles.axes2);
        imshow(im_noise);
        % generates Poisson noise from the data instead of adding artificial noise to the data
        
    case 3
        im_noise= imnoise(im_colored,'salt & pepper');
        axes(handles.axes2);
        imshow(im_noise);

        %adds salt and pepper noise, with default noise density 0.05. This affects approximately 5% of pixels.
    case 4
        im_noise = imnoise(im_colored,'speckle');
        axes(handles.axes2);
        imshow(im_noise);

    otherwise
end


% --- Executes during object creation, after setting all properties.
function AddNoise_CreateFcn(hObject, ~, handles)
% hObject    handle to AddNoise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in StartStop.
function StartStop_Callback(hObject, eventdata, handles)
% hObject    handle to StartStop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes2);
vid=videoinput('winvideo',2);
hImage=image(zeros(352,288,3),'Parent',handles.axes3);
preview(vid,hImage);

% --- Executes on button press in CaptureImage.
function CaptureImage_Callback(hObject, eventdata, handles)
% hObject    handle to CaptureImage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in startAcquisition.
function startAcquisition_Callback(hObject, eventdata, handles)
% hObject    handle to startAcquisition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Contour.
function Contour_Callback(hObject, eventdata, handles)
% hObject    handle to Contour (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Here we find contours of connected object in the image
global im
im_grey = rgb2gray(im);% convert the image to a grayscale image
BW = imbinarize(im_grey); % Concert the grayscale image to a binary one 
[B,L] = bwboundaries(BW,'noholes');
axes(handles.axes2)
imshow(im)
hold on
%imshow(label2rgb(L, @jet, [.5 .5 .5]))
hold on
for k = 1:length(B)
   boundary = B{k};
   plot(boundary(:,2), boundary(:,1), 'g', 'LineWidth', 2)
end


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(~, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function uipanel13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to uipanel13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on selection change in PointDescriptors.
function PointDescriptors_Callback(hObject, eventdata, handles)
% hObject    handle to PointDescriptors (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns PointDescriptors contents as cell array
%        contents{get(hObject,'Value')} returns selected item from PointDescriptors
%% Here we extract HARRIS, FAST , SURF and SIFT 
global im_colored
contents = get (hObject,'Value');

switch contents 
    case 1
        %Extract HARRIS
        points = detectHarrisFeatures(im_colored);
        [features, valid_points] = extractFeatures(im_colored, points);
        axes(handles.axes2);
        imshow(im_colored); hold on;
        plot(valid_points.selectStrongest(10),'showOrientation',true);
    case 2
        %Extract FAST
        points = detectFASTFeatures(im_colored);
        [features, valid_points] = extractFeatures(im_colored, points);
        axes(handles.axes2);
        imshow(im_colored); hold on;
        plot(valid_points.selectStrongest(10),'showOrientation',true);  
    case 3
        %Extract SURF features
        points = detectSURFFeatures(im_colored);
        [features, valid_points] = extractFeatures(im_colored, points);
        axes(handles.axes2);
        imshow(im_colored); hold on;
        plot(valid_points.selectStrongest(10),'showOrientation',true);
    case 4
        
        %Apply SIFT 
        
    otherwise 
end 
      

% --- Executes during object creation, after setting all properties.
function PointDescriptors_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PointDescriptors (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in BrowseVideo.
function BrowseVideo_Callback(hObject, eventdata, handles)
% hObject    handle to BrowseVideo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global filename
[chosenfile, chosenpath] = uigetfile('*.avi', 'Select a video');
filename = fullfile(chosenpath, chosenfile);
 


% --- Executes on button press in StartVideo.
function StartVideo_Callback(hObject, eventdata, handles)
% hObject    handle to StartVideo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global filename
    obj = VideoReader(filename);
    ax = handles.axes5;
    while hasFrame(obj)
        vidFrame = readFrame(obj);
        image(vidFrame, 'Parent', ax);
        set(ax, 'Visible', 'off');
        pause(1/obj.FrameRate);
    end
   
    sobelVid = edge(obj,'Sobel'); 
    ax1 = handles.axes6;
    while hasFrame(sobelVid)
        vidFrame1 = readFrame1(sobelVid );
        image(vidFrame1, 'Parent', ax1);
        set(ax1, 'Visible', 'off');
        pause(1/obj.FrameRate);
    end

    


% --- Executes on selection change in popupmenu15.
function popupmenu15_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu15 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu15


% --- Executes during object creation, after setting all properties.
function popupmenu15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu14.
function popupmenu14_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu14 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu14


% --- Executes during object creation, after setting all properties.
function popupmenu14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu13.
function popupmenu13_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu13 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu13


% --- Executes during object creation, after setting all properties.
function popupmenu13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
