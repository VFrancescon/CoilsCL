#include "cameraGeneric.hpp"
#include "precomputation.hpp"
#include "HCoilMiddlewareLib/HCoilMiddlewareLib.hpp"



double upperError  = 5.6e3;
double lowError = 2e3;

Mat IntroducerMask(Mat src){
    Mat src_GRAY, element;
    //create a greyscale copy of the image
    // flip(src, src, 1);

    cvtColor(src, src_GRAY, COLOR_BGR2GRAY);
    
    //apply blur and threshold so that only the tentacle is visible
    blur(src_GRAY, src_GRAY, Size(5,5));
    threshold(src_GRAY, src_GRAY, threshold_low, threshold_high, THRESH_BINARY_INV); 
    
    element = getStructuringElement(MORPH_DILATE, Size(3,3) );
    dilate(src_GRAY, src_GRAY, element);
    
 
    bitwise_not(src_GRAY, src_GRAY);

    return src_GRAY;

}
int meanError(std::vector<double> &desired, std::vector<double> &observed){
    double d_error, sum = 0;
    int error;
    for(size_t i = 0; i < desired.size(); i++){
        sum += desired[i] - observed[i];
    }
    d_error = sum / desired.size();

    return (int) (d_error * 1000);
}





template<typename T>
double avgVect(std::vector<T> inputVec){
    double avg, sum = 0;

    for(auto i : inputVec){
        sum += i;
    }
    avg = sum / inputVec.size();
    return avg;
}


std::vector<double> computeAngles(std::vector<Point> Joints){
    std::vector<double> angles;
    std::vector<Point> vects;

    Joints.insert(Joints.begin(), Point(Joints[0].x, 0));
    for(int i = 1; i < Joints.size(); i++){
        vects.push_back(Point{Joints[i].x - Joints[i-1].x, Joints[i].y - Joints[i-1].y}  );
    }
    for(int i = 0; i < vects.size()-1; i++){
        double dproduct = vects[i].dot(vects[i+1]);
        double nproduct = norm(vects[i]) * norm(vects[i+1]);
        double th = acos(dproduct/nproduct);
        angles.push_back(th * 180 / M_PI);
    }

    return angles;

}


std::vector<Point> computeIdealPoints(Point p0, std::vector<double> desiredAngles){
    std::vector<Point> ideal;

    ideal.push_back(p0);
    for(int i = 1; i < desiredAngles.size(); i++){
        double angle = 0;
        for( int k = 0; k < i; k++) angle += desiredAngles[k];
        int xdiff = (link_lenght+15) * sin(angle * M_PI / 180);
        int ydiff = (link_lenght+15) * cos(angle * M_PI / 180);
        Point pn = Point{ (int) (ideal[i-1].x + xdiff), (int) ( ideal[i-1].y + ydiff )}; 
        ideal.push_back(pn);
    }

    return ideal;
}


inline bool file_exists (const std::string& name) {
    struct stat buffer;   
    return (stat (name.c_str(), &buffer) == 0); 
}




std::vector<Point> findJoints(Mat post_img_masked, std::vector<std::vector<Point>> &contours){
    

    Mat contours_bin;
    // std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    //find contours, ignore hierarchy
    findContours(post_img_masked, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    contours_bin = Mat::zeros(post_img_masked.size(), CV_8UC1);

    //draw contours and fill the open area
    drawContours(contours_bin, contours, -1, Scalar(255,255,255), cv::FILLED, LINE_8, hierarchy);
    //empty matrix. Set up to 8-bit 1 channel data. Very important to set up properly.
    Mat skeleton = Mat::zeros(post_img_masked.rows, post_img_masked.rows, CV_8U);
    
    //take the filled contour and thin it using Zhang Suen method. Only works with 8-bit 1 channel data. 
    ximgproc::thinning(contours_bin, skeleton, 0);
    
    contours.clear();
    hierarchy.clear();
    findContours(skeleton, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

    std::vector<Point> cntLine;
    findNonZero(skeleton, cntLine);
    std::sort(cntLine.begin(), cntLine.end(), yWiseSort);
    // std::reverse(cntLine.begin(), cntLine.end());
    
    std::vector<Point> Joints;
    int jointCount = (int) cntLine.size() / link_lenght;
    
    
    if(jointCount){
        for(int i = 0; i < jointCount; i++){
            Joints.push_back(cntLine[link_lenght*(i)]);
        }
    }
    std::reverse(Joints.begin(), Joints.end());

    return Joints;
}

void adjustStiffness(std::vector<Link> &iLinks, double EMulitplier);
Vector3d CalculateField(std::vector<Link> &iLinks, std::vector<Joint> &iJoints, std::vector<PosOrientation> &iPosVec);

int main(int argc, char* argv[]){
    int jointEff = 5;
    int jointNo = jointEff+1;

    //timesteps are equal to joint no
    int timesteps = jointEff;  
    // Vector3d reconciliationAngles = Vector3d{-90, 0, 90};
    double EMulitplier = 5;
    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * PRECOMPUTATION FOR EACH TIMESTEP BEGINS HERE  *
     *                                               *
     *                                               *
     * * * * * * * * * * * * * * * * * * * * * * * * */
    std::vector<Vector3d> AppliedFields;

    std::vector<int> DesiredAngles(jointNo);
    DesiredAngles[0] = 10;
    DesiredAngles[1] = 5;
    DesiredAngles[2] = 5;
    DesiredAngles[3] = 10;
    DesiredAngles[4] = 10;
    DesiredAngles[jointEff] = 0;

    std::vector<Vector3d> Magnetisations(jointNo);
    Magnetisations[0] = Vector3d(-0.0011,0,-0.0028);
    Magnetisations[1] = Vector3d(-0.0028,0,0.001);
    Magnetisations[2] = Vector3d(0,0,-0.003);
    Magnetisations[3] = Vector3d(-0.003,0,0);
    Magnetisations[4] = Vector3d(0,0,-0.003);
    Magnetisations[jointEff] = Vector3d(0,0,0);

    std::vector<PosOrientation> iPosVec(jointNo);
    std::vector<Joint> iJoints(jointNo);
    for(int i = 0; i < jointNo; i++){
        iJoints[i].assignPosOri(iPosVec[i]);
    }

    for(int i = 0; i < jointNo; i++){
        iJoints[i].q = Vector3d(0,DesiredAngles[i],0);
        iJoints[i].LocMag = Magnetisations[i];
    }




    //create vector of links for properties
    std::vector<Link> iLinks(jointEff) ;
    adjustStiffness(iLinks, EMulitplier);
    

    // Vector3d field = RotateField(solution, reconciliationAngles);
    Vector3d field = CalculateField(iLinks, iJoints, iPosVec);
    field(1) = 0;
    


    /**
     * MIDDLEWARE SECTION BELOW
     * 
     */
    MiddlewareLayer mid(true);
    mid.set3DField(field);

    Mat pre_img, post_img, intr_mask;
    Pylon::PylonInitialize();
    Pylon::CImageFormatConverter formatConverter;
    formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
    Pylon::CPylonImage pylonImage;
    Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice() );
    camera.Open();
    Pylon::CIntegerParameter width     ( camera.GetNodeMap(), "Width");
    Pylon::CIntegerParameter height    ( camera.GetNodeMap(), "Height");
    Pylon::CEnumParameter pixelFormat  ( camera.GetNodeMap(), "PixelFormat");
    Pylon::CFloatParameter(camera.GetNodeMap(), "ExposureTime").SetValue(20000.0);
    Size frameSize= Size((int)width.GetValue(), (int)height.GetValue());
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    width.TrySetValue(1920, Pylon::IntegerValueCorrection_Nearest);
    height.TrySetValue(1216, Pylon::IntegerValueCorrection_Nearest);
    Pylon::CPixelTypeMapper pixelTypeMapper( &pixelFormat);
    Pylon::EPixelType pixelType = pixelTypeMapper.GetPylonPixelTypeFromNodeValue(pixelFormat.GetIntValue());
    
    
    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    Pylon::CGrabResultPtr ptrGrabResult;
    camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
    const uint8_t* preImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
    formatConverter.Convert(pylonImage, ptrGrabResult);
    pre_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());
    
    int rrows = pre_img.rows / 2;
    int rcols = pre_img.cols * 3 / 8; 
    
    /**
     * VIDEO OUTPUT WRITE
     * 
     */
    std::string outputPath = "C_PROTOTYPE.avi";

    while(file_exists(outputPath)){
        outputPath += "_1";
    }


    VideoWriter video_out(outputPath, VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, 
                Size(rcols, rrows));



    // resize(pre_img, pre_img, Size(rcols, rrows), INTER_LINEAR);
    // Mat pre_img1 = Mat::zeros(Size(rcols, rrows), CV_8UC3);
    // intr_mask = IntroducerMask(pre_img1);
    intr_mask = IntroducerMask(pre_img);
    int jointsCached = 0;
    Point p0 = Point{-2000,2000};
    double bx_add = 0, bz_add = 0;
    std::cout << "Ready to go. Press enter";
    std::cin.get();

    while(camera.IsGrabbing()){
        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);
        post_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

        if(post_img.empty())
        {
            break;
        }
        resize(post_img, post_img, Size(rcols, rrows), INTER_LINEAR);
        Mat post_img_grey, post_img_th;
        Mat post_img_masked = Mat::zeros(Size(rrows,rcols), CV_8UC1);

        cvtColor(post_img, post_img_grey, COLOR_BGR2GRAY);
        blur(post_img_grey, post_img_grey, Size(5,5));
        threshold(post_img_grey, post_img_th, threshold_low, threshold_high, THRESH_BINARY_INV);
        post_img_th.copyTo(post_img_masked);

        std::vector<Point> Joints;
        std::vector<std::vector<Point> > contours;

        Joints = findJoints(post_img_masked, contours);
        int JointsObserved = Joints.size();
        for(auto i: Joints){
            circle(post_img, i, 4, Scalar(255,0,0), FILLED);        
        }
        drawContours(post_img, contours, -1, Scalar(255,255,0));
        
        std::vector<double> angles; 
        std::vector<double> desiredAngles = std::vector<double>(DesiredAngles.begin(), DesiredAngles.end()-1);
        std::vector<Point> idealPoints;
        if(p0 == Point{-2000,2000}) p0 = Joints[0];

        idealPoints = computeIdealPoints(p0, desiredAngles);
        angles = computeAngles(Joints);
        for(int i = 0; i < idealPoints.size()-1; i++){
            line(post_img, idealPoints[i], idealPoints[i+1], Scalar(0,0,255));
            circle(post_img, idealPoints[i], 2, Scalar(255,0,0));
        }



        // if(JointsObserved != jointsCached){

        jointsCached = JointsObserved;
        std::vector<double> dAngleSlice = std::vector<double>(desiredAngles.end()-angles.size(), desiredAngles.end());         
        // std::vector<double> dAngleSlice = desiredAngles;
        int error = meanError(dAngleSlice, angles);
        std::cout << "\n\n---------------------------------------------------------\n\n";
        

        //Fetched error e
        //Now we have N scenarios
        //And thresholds LowS, HighS
        //And adjustment factor P
        //Scenario 1. -LowS < e < +LowS -> Do nothing
        //Scenario 2. -HighS < e < -LowS -> Field - P
        //Scenario 3. e < -HighS -> K--
        //Scenario 4. +LowS > e > +HighS -> Field + P
        //Scenario 5. e > +HighS -> K++


        //Slightly less verbose
        //if e < 0: signFlag = -1
        //else signFlag = 1
        //then e = abs(e)
        //Scenario 1. e < LowS -> Do Nothing
        //Scenario 2. LowS < e < HighS -> Field + P*signFlag
        //Scenario 3. e > HighS -> K += signFlag
        int signFlag = (error > 0) ? -1 : 1;
        std::cout << "Error " << error << "\n";
        error = abs(error);

        if( error < lowError ) {
            imshow("Post", post_img);
            video_out.write(post_img);
            char c= (char)waitKey(0);
            continue;
        } else if ( error > lowError && error < upperError){
            field += field * 0.1 * signFlag;
            std::cout << "Adjusting field\n";
        } else if ( error > upperError){
            EMulitplier += signFlag;
            adjustStiffness(iLinks, EMulitplier);
            CalculateField(iLinks, iJoints, iPosVec);
            std::cout << "Adjusting E\n";
        }

        std::cout << "E: " << EMulitplier << " applied field:\n" << field << "\n";
        
        if( abs(field(0)) > 20 && abs(field(2)) > 15 && abs(field(1)) > 20) break;
        if( EMulitplier < 0 ) break;


        mid.set3DField(field);


        imshow("Post", post_img);
        video_out.write(post_img);
        char c= (char)waitKey(5e2);
        if(c==27) break;
        
    }
    video_out.release();
    mid.~MiddlewareLayer();
    // destroyAllWindows();
    // Pylon::PylonTerminate();
    return 0;
}


/***********************************************
 * Real time adjustment functions below
 * 
 * 
************************************************/

void adjustStiffness(std::vector<Link> &iLinks, double EMulitplier){
    dfltValues MechPpts;

    for(int i = 0; i < iLinks.size(); i++){
        iLinks[i].dL = MechPpts.len;
        iLinks[i].d = MechPpts.d;
        iLinks[i].E = MechPpts.E * EMulitplier;
        iLinks[i].v = MechPpts.v;

    }

}


Vector3d CalculateField(std::vector<Link> &iLinks, std::vector<Joint> &iJoints, 
    std::vector<PosOrientation> &iPosVec){
    MatrixXd KStacked;
    KStacked = EvaluateK(iLinks);
    int jointNo = iLinks.size();
    VectorXd AnglesStacked;
    AnglesStacked = StackAngles(iJoints);
    

    DirectKinematics(iPosVec, iJoints, iLinks);
    MatrixXd Jacobian, Jt;
    Jacobian = EvaluateJacobian(iPosVec);
    Jt = Jacobian.transpose();

    MatrixXd FieldMap;
    FieldMap = MagtoFieldMap(iJoints);

    MatrixXd RHS = Jt*FieldMap;
    AnglesStacked = AnglesStacked * M_PI / 180;
    MatrixXd LHS = KStacked * AnglesStacked;
    
    MatrixXd solution = RHS.completeOrthogonalDecomposition().solve(LHS);
    return solution * 1000;
}

/**************************************************************
 * 
 * 
 * PRECOMPUTATION FUNCTIONS DOWN HERE
 * 
 * 
*****************************************************************/

/**
 * @brief Precomputation Function. Evaluates the stiffness Matrix of the system given the links
 * 
 * @param iLinks Vector of Link structs containing all Link mechanical properties
 * @return MatrixXd Diagonally Stacked 3.n x 3.n stiffness matrix
 */
MatrixXd EvaluateK(std::vector<Link> &iLinks){
	std::vector<Matrix3d> K_vec;

    for(int i = 0; i < iLinks.size(); i++){
		double lRadius = iLinks[i].d / 2;
		double I = M_PI_4 * lRadius * lRadius * lRadius * lRadius;
		double G = iLinks[i].E / (2* (iLinks[i].v + 1) );
		double J = M_PI_2 * lRadius * lRadius * lRadius * lRadius;
		double Kb = iLinks[i].E*I/iLinks[i].dL;
		double Kt = G*J/iLinks[i].dL;
		Matrix3d K = Matrix3d::Zero();
		K(0,0) = Kb;
		K(1,1) = Kb;
		K(2,2) = Kt;
		K_vec.push_back(K);
        // std::cout << "I " << i << "\nK\n" << K << "\n";
	}
	
	MatrixXd KDiagonal;
	KDiagonal = StackDiagonals(K_vec);
    return KDiagonal;
}

/**
 * @brief Precomputation Function. Precalculates axis unit vectors and positional vectors for Jacobian computation
 * 
 * @param iPosVec vector containing joint positions and axis orientation
 * @param iJoints vector containing the joint's Transform matrix and pointers to iPosVec
 * @param iLinks vector containing the links and their mechanical properties
 */
void DirectKinematics(std::vector<PosOrientation> &iPosVec, std::vector<Joint> &iJoints, std::vector<Link> &iLinks){
	iJoints[0].Rotation = Matrix3d::Identity();
	iJoints[0].pLocal = Vector3d::Zero();

    int jointEff = (int) iJoints.size();

	for(int i = 1; i < jointEff; i++){
		iJoints[i].Rotation = RotationZYX(iJoints[i-1].Rotation, iJoints[i-1].q);
		iJoints[i].pLocal = iJoints[i-1].pLocal + iJoints[i].Rotation * Vector3d(0,0, -iLinks[i-1].dL);
    }

    for(int i = 0; i < jointEff; i++){
		
		iPosVec[i].p = iJoints[i].pLocal;
		
		iPosVec[i].z(placeholders::all,0) = iJoints[i].Rotation * Vector3d::UnitX();
		
		iPosVec[i].z(placeholders::all,1) = AngleAxisd( iJoints[i].q(0) * M_PI / 180, Vector3d::UnitX() ) *  
							iJoints[i].Rotation * Vector3d::UnitY();
		
		iPosVec[i].z(placeholders::all,2) = AngleAxisd( iJoints[i].q(1) * M_PI / 180, Vector3d::UnitY() ) * 
							AngleAxisd( iJoints[i].q(0) * M_PI / 180, Vector3d::UnitX() ) * 
							iJoints[i].Rotation * Vector3d::UnitZ();	

    }
    return;
}

/**
 * @brief Precomputation Function. Evaluates the jacobian using orientations and positions containing in a vector of points
 * 
 * @param iPosVec vector contains Positions and Orientations
 * @param jointEff Number of effective joints (discount end effector basically)
 * @return MatrixXd size (joint)*6 x (jointEff)*3 containing the full Jacobian computed
 */
MatrixXd EvaluateJacobian(std::vector<PosOrientation> &iPosVec){
	/**
	 * @note
	 * 
	 * Given J = [ 	J00 J01
	 * 				J10 J11 	]
	 * Where J_xy = [Jp_xy
	 * 				Jo_xy]
	 * 
	 * In the loops below, 	i tracks y
	 * 						k tracks x
	 * 
	 * Also the 'stacking' of the full jacobian is actually done by 
	 * initialising an empty Mat of the correct size and filling in the blocks
	 * stacking in the Matrix algebra library we use is possible, but
	 * a pain, so filling is good enough, probably.
	 */
    
    int jointEff = (int) iPosVec.size() -1;
	Matrix3d Jp, Jo;
	MatrixXd Jacobian(jointEff*6, jointEff*3);
	for(int i = 0; i < jointEff; i++){
		//i goes vertically
		for(int k = 0; k < jointEff; k++){
			//k goes horizontally
			if( k > i ) {
				Jp = Matrix3d::Zero();
				Jo = Matrix3d::Zero();
			} else{
                try{
                    Vector3d pDiff = iPosVec[i+1].p - iPosVec[k].p;
                    std::vector<Vector3d> z1{iPosVec[k].z(placeholders::all,0), iPosVec[k].z(placeholders::all,1), iPosVec[k].z(placeholders::all,2)};
                    std::vector<Vector3d> ZcrossP{z1[0].cross(pDiff), z1[1].cross(pDiff), z1[2].cross(pDiff)};
                    Jp << ZcrossP[0] , ZcrossP[1], ZcrossP[2];
                    Jo << z1[0], z1[1], z1[2];
                    
                    // std::cout << "i: " << i << " k: " << k << "\n";
                    // std::cout << "Jp\n" << Jp << "\n";
                    // std::cout << "Jo\n" << Jo << "\n";	
                    
                    }
                    
                catch(std::exception &e){
                    std::cout << "caught error at e: " << e.what() << "\n";
                    throw;
                }
			}
			MatrixXd Jn( Jp.rows() + Jo.rows(), Jp.cols());	
			Jn << Jp, 
				Jo;
			Jacobian(seq(0+i*6, 5+i*6), seq(0+k*3,2+k*3)) = Jn;
		}
	}
	return Jacobian;
}

/**
 * @brief Precomputation function. Calculates a Vertically stacked Map from Magnetisation to Field
 * 
 * @param iJoints ordered vector of joints, each containing the set magnetisation
 * @return MatrixXd 6*n x 3 matrix of vertically stacked 0(3x3) and maps. See Llyod 2020 and Salmanipour/Diller 2018
 */
MatrixXd MagtoFieldMap(std::vector<Joint> &iJoints){
    MatrixXd Map(6*(iJoints.size()-1), 3);
    Map = MatrixXd::Zero(6*(iJoints.size()-1), 3);
    Matrix3d Zeros = Matrix3d::Zero();
    for(int i = 0; i < iJoints.size()-1; i++){
        iJoints[i].GlobMag = iJoints[i+1].Rotation * iJoints[i].LocMag;
        Matrix3d Skewd = SkewMatrix(iJoints[i].GlobMag);
        Map( seqN(3+6*i, 3), seqN(0, 3)  ) = -Skewd;         
    }
    // std::cout << "Magtofield map\n" << Map << "\n";
    // std::cout << "Sizing: " << Map.rows() << "x" << Map.cols() << "\n";
    // std::cout << "Size of ijoints " << iJoints.size() << "\n";
    return Map;
}


Matrix3d SkewMatrix(Vector3d src){
    Matrix3d Skew;
    Skew = Matrix3d::Zero();
    Skew << 0, -src[2], src[1],
            src[2], 0, -src[0],
            -src[1], src[0], 0;
    return Skew;
}

VectorXd StackAngles(std::vector<Joint>& iJoints){
    int jointEff = iJoints.size();
    VectorXd stacked;

    if(jointEff == 2){
        stacked = iJoints[0].q;
    }
    else{
        stacked = VerticalStack(iJoints[0].q, iJoints[1].q);
        for(int i = 2; i < jointEff-1; i++){
            stacked = VerticalStack(stacked, iJoints[i].q);
        }
    }

    // stacked = stacked * M_PI / 180;

    return stacked;
}

/**
 * @brief Utility Function. Vertically Stacks a 3x3 Matrix onto an existing Xx3 Matrix
 * 
 * @param M1 Matrix of any number of rows, but 3 columns
 * @param M2 3x3 Matrix to stack below
 * @return MatrixXd resultatant of stack
 */
MatrixXd VerticalStack(MatrixXd M1, MatrixXd M2){
    MatrixXd Stack(M1.rows() + M2.rows(), M1.cols());
    Stack << M1, M2;
    return Stack;
}

/**
 * @brief Utility Function. Creates a diagonal matrix by stacking 3x3 matrices contained in vector matrices
 * 
 * @param matrices vector containing n 3x3 matrices
 * @return MatrixXd - 3*n, 3*n Matrix containing the diagonal stack
 */
MatrixXd StackDiagonals(std::vector<Matrix3d> matrices){
	MatrixXd diagonal(matrices.size()*3, matrices.size()*3);
    diagonal = MatrixXd::Zero(diagonal.rows(), diagonal.cols());
	for(size_t i = 0; i < matrices.size(); i++){
		
		diagonal( seq(i*3, 2+i*3), seq(i*3, 2+i*3)) = matrices[i];
	
	}
	// std::cout << "Diagonal evaluated\n" << diagonal << "\n";
    return diagonal;
}

/**
 * @brief Explicit reimplementation or RotationZYX. Applies the same principle to rotate the applied field.
 * Rotates field by Z and then X
 * 
 * @param field 3d vector containing applied field in all directions.
 * @param rotationAngles Angles required to rotate, leave Y blank
 * @return Vector3d 
 */
Vector3d RotateField(Vector3d field, Vector3d rotationAngles){
    double AngleZ = rotationAngles(2) * M_PI / 180;
    double AngleX = rotationAngles(0) * M_PI / 180;

    return AngleAxisd(AngleZ, Vector3d::UnitZ()) 
                * AngleAxisd(AngleX, Vector3d::UnitX()) * field;
}

/**
 * @brief Utility Function. Rotates matrix src by angles in vector jointAngles in the ZYX order.
 * 
 * @param src matrix containing original position
 * @param jointAngles column vector containing rotations. (X,Y,Z) 
 * @return Matrix3d, Rotated matrix after being multiplied by angles jointAngles 
 */
Matrix3d RotationZYX(Matrix3d src, Vector3d jointAngles){
	double AngleZ = jointAngles(2) * M_PI / 180;
	double AngleY = jointAngles(1) * M_PI / 180;
	double AngleX = jointAngles(0) * M_PI / 180;
	
	return src * AngleAxisd(AngleZ, Vector3d::UnitZ())
		* AngleAxisd(AngleY, Vector3d::UnitY())
		* AngleAxisd(AngleX, Vector3d::UnitX());
}

