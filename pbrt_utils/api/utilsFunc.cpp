#include<api/utilsFunc.h>

std::string generateFilenames(std::string filenames,std::string suffix = "_user"){
    std::string newfilenames = filenames.substr(0, filenames.length()-5)+suffix + ".pbrt";
    return newfilenames;
}
// this need to be overload to different versions
//hhq
std::string generatePbrtFile(RealisticCameraParam RC, std::string filenames){
    std::string newfilenames;
    // newfilenames = filenames.substr(0, filenames.length()-5)+"_user.pbrt";
    newfilenames = generateFilenames(filenames,"_user");
    std::ifstream infile(filenames);
    std::ofstream outfile(newfilenames);

    int findCamera = 0;//0:have not findCamera 1:finding 2:have find 3:finding another Camera
    int lineAfterCamera = 0;
    std::string temp;
    std::string templast;
    ChangeLookAt(RC,infile,outfile);
    while (!infile.eof())
    {
        getline(infile,temp); 
        //first: judge the state,be sure findCamera will only be changed per line
        if("Film"==temp.substr(0,4)){
            //special judge
            std::string diagonal ="    \"float diagonal\" [ "+std::to_string(RC.diagonal)+" ]";
            temp.append(diagonal);
            outfile << temp << std::endl;
            continue;
        }
        if("Camera"==temp.substr(0,6)){
            if(0 == findCamera){
                findCamera = 1;
            }
            else if(2 == findCamera){
                findCamera = 3;
            }
        }
        else if((1 == findCamera || 3 == findCamera) && ' '!=temp[0] && '"'!=temp[0])
        {
            findCamera = 2;
        }

        //then:output file
        if( (0 ==findCamera) || (2 == findCamera) ){
            outfile << temp << std::endl;
        }
        else if(3 == findCamera){
            //do not output f|| now, because I don't know why there would be two Camera setting
            continue;
        }
        else if (1 == findCamera)
        {
            //TODO, if you need to overload
            while(1){
                if(0 == lineAfterCamera){
                    temp = "Camera \"realistic\"";
                }
                else if (1 == lineAfterCamera)
                {
                    temp = "    \"float shutteropen\" [ "+ std::to_string(RC.shutteropen)  +" ]";
                }
                else if (2 == lineAfterCamera)
                {
                    temp = "    \"float shutterclose\" [ "+ std::to_string(RC.shutterclose)  +" ]";
                }
                else if (3 == lineAfterCamera)
                {
                    temp ="    \"string lensfile\" [ \""+RC.lensfile+"\" ]";
                }
                else if (4 == lineAfterCamera)
                {
                    temp ="    \"float aperturediameter\" [ "+std::to_string(RC.aperturediameter)+" ]";
                }
                else if (5 == lineAfterCamera)
                {
                    temp ="    \"float focusdistance\" [ "+std::to_string(RC.focusdistance)+" ]";
                }
                else if (6 == lineAfterCamera)
                {
                    if("circular"!=RC.aperture){
                        temp ="    \"string aperture\" [ "+RC.aperture+" ]";
                        outfile << temp << std::endl;
                        break;
                    }
                    else{
                        break;
                    }
                }
                lineAfterCamera++;
                outfile << temp << std::endl;
            }
            //after output, delete other lines until all the ||iginal Camera msg is removed 
            findCamera = 3;
        }
    }
    infile.close();
    outfile.close();
    return newfilenames;
}
std::string generatePbrtFile(PerspectiveCameraParam PC, std::string filenames){

    std::string newfilenames;
    newfilenames = generateFilenames(filenames,"_user");
    std::ifstream infile(filenames);
    std::ofstream outfile(newfilenames);

    int findCamera = 0;//0:have not findCamera 1:finding 2:have find 3:finding another Camera
    int lineAfterCamera = 0;
    std::string temp;
    std::string templast;
    while (!infile.eof())
    {
        getline(infile,temp); 
        //first: judge the state,be sure findCamera will only be changed per line
        if("Camera"==temp.substr(0,6)){
            if(0 == findCamera){
                findCamera = 1;
            }
            else if(2 == findCamera){
                findCamera = 3;
            }
        }
        else if((1 == findCamera || 3 == findCamera) && ' '!=temp[0]  && '"'!=temp[0])
        {
            findCamera = 2;
        }

        //then:output file
        if(0 ==findCamera || 2 == findCamera ){
            outfile << temp << std::endl;
        }
        else if(3 == findCamera){
            //do not output f|| now, because I don't know why there would be two Camera setting
            continue;
        }
        else if (1 == findCamera)
        {
            //TODO, if you need to overload
            while(1){
                if(0 == lineAfterCamera){
                    temp = "Camera \"perspective\"";
                }
                else if (1 == lineAfterCamera)
                {
                    temp = "    \"float shutteropen\" [ "+ std::to_string(PC.shutteropen)  +" ]";
                }
                else if (2 == lineAfterCamera)
                {
                    temp = "    \"float shutterclose\" [ "+ std::to_string(PC.shutterclose)  +" ]";
                }
                else if (3 == lineAfterCamera)
                {
                temp ="    \"float fov\" [ "+std::to_string(PC.fov)+" ]";
                }
                else if (4 == lineAfterCamera)
                {
                    temp ="    \"float lensradius\" [ "+std::to_string(PC.lensradius)+" ]";
                }
                else if (5 == lineAfterCamera)
                {   
                    if(-0.9<PC.focaldistance){
                        temp ="    \"float focaldistance\" [ "+std::to_string(PC.focaldistance)+" ]";
                    }
                    else{

                    }
                }
                else if (6 == lineAfterCamera)
                {
                    if(-0.9<PC.frameaspectratio){
                        temp ="    \"float frameaspectratio\" [ "+std::to_string(PC.frameaspectratio)+" ]";
                    }
                    else{
                    }
                }
                else if (7 == lineAfterCamera)
                {
                    if(-0.9<PC.screenwindow){
                        temp ="    \"float screenwindow\" [ "+std::to_string(PC.screenwindow)+" ]";
                        outfile << temp << std::endl;
                        break;
                    }
                    else{
                        break;
                    }
                }
                lineAfterCamera++;
                outfile << temp << std::endl;
            }
            //after output, delete other lines until all the ||iginal Camera msg is removed 
            findCamera = 3;
        }
    }
    infile.close();
    outfile.close();
    return newfilenames;
}
std::string generatePbrtFile(OrthographicCameraParam OC, std::string filenames){
    std::string newfilenames;
    newfilenames = generateFilenames(filenames,"_user");
    std::ifstream infile(filenames);
    std::ofstream outfile(newfilenames);

    int findCamera = 0;//0:have not findCamera 1:finding 2:have find 3:finding another Camera
    int lineAfterCamera = 0;
    std::string temp;
    std::string templast;
    while (!infile.eof())
    {
        getline(infile,temp); 
        //first: judge the state,be sure findCamera will only be changed per line
        if("Camera"==temp.substr(0,6)){
            if(0 == findCamera){
                findCamera = 1;
            }
            else if(2 == findCamera){
                findCamera = 3;
            }
        }
        else if((1 == findCamera || 3 == findCamera) && ' '!=temp[0] && '"'!=temp[0])
        {
            findCamera = 2;
        }

        //then:output file
        if(0 ==findCamera || 2 == findCamera ){
            outfile << temp << std::endl;
        }
        else if(3 == findCamera){
            //do not output f|| now, because I don't know why there would be two Camera setting
            continue;
        }
        else if (1 == findCamera)
        {
            //TODO, if you need to overload
            while(1){
                if(0 == lineAfterCamera){
                    temp = "Camera \"||thographic\"";
                }
                else if (1 == lineAfterCamera)
                {
                    temp = "    \"float shutteropen\" [ "+ std::to_string(OC.shutteropen)  +" ]";
                }
                else if (2 == lineAfterCamera)
                {
                    temp = "    \"float shutterclose\" [ "+ std::to_string(OC.shutterclose)  +" ]";
                }
                else if (3 == lineAfterCamera)
                {
                    temp ="    \"float lensradius\" [ "+std::to_string(OC.lensradius)+" ]";
                }
                else if (4 == lineAfterCamera)
                {
                    if(-0.9<OC.focaldistance){
                        temp ="    \"float focaldistance\" [ "+std::to_string(OC.focaldistance)+" ]";
                    }
                    else{

                    }
                }
                else if (5 == lineAfterCamera)
                {
                    if(-0.9<OC.frameaspectratio){
                        temp ="    \"float frameaspectratio\" [ "+std::to_string(OC.frameaspectratio)+" ]";
                    }
                    else{
                    }
                }
                else if (6 == lineAfterCamera)
                {
                    if(-0.9<OC.screenwindow){
                        temp ="    \"float screenwindow\" [ "+std::to_string(OC.screenwindow)+" ]";
                        outfile << temp << std::endl;
                        break;
                    }
                    else{
                        break;
                    }
                }
                lineAfterCamera++;
                outfile << temp << std::endl;
                temp = "";
            }
            //after output, delete other lines until all the ||iginal Camera msg is removed 
            findCamera = 3;
        }
    }
    infile.close();
    outfile.close();
    return newfilenames;
}
std::string generatePbrtFile(SphericalCameraParam SC, std::string filenames){
    std::string newfilenames;
    newfilenames = generateFilenames(filenames,"_user");
    std::ifstream infile(filenames);
    std::ofstream outfile(newfilenames);

    int findCamera = 0;//0:have not findCamera 1:finding 2:have find 3:finding another Camera
    int lineAfterCamera = 0;
    std::string temp;
    std::string templast;
    while (!infile.eof())
    {
        getline(infile,temp); 
        //first: judge the state,be sure findCamera will only be changed per line
        if("Camera"==temp.substr(0,6)){
            if(0 == findCamera){
                findCamera = 1;
            }
            else if(2 == findCamera){
                findCamera = 3;
            }
        }
        else if((1 == findCamera || 3 == findCamera) && ' '!=temp[0] && '"'!=temp[0])
        {
            findCamera = 2;
        }

        //then:output file
        if(0 ==findCamera || 2 == findCamera ){
            outfile << temp << std::endl;
        }
        else if(3 == findCamera){
            //do not output f|| now, because I don't know why there would be two Camera setting
            continue;
        }
        else if (1 == findCamera)
        {
            //TODO, if you need to overload
            while(1){
                if(0 == lineAfterCamera){
                    temp = "Camera \"spherical\"";
                }
                else if (1 == lineAfterCamera)
                {
                    temp = "    \"float shutteropen\" [ "+ std::to_string(SC.shutteropen)  +" ]";
                }
                else if (2 == lineAfterCamera)
                {
                    temp = "    \"float shutterclose\" [ "+ std::to_string(SC.shutterclose)  +" ]";
                }
                else if (3 == lineAfterCamera)
                {
                    if("equalarea"< SC.mapping){
                        temp ="    \"string mapping\" [ "+SC.mapping+" ]";
                        outfile << temp << std::endl;
                        break;
                    }
                    else{
                        break;
                    }
                }
                lineAfterCamera++;
                outfile << temp << std::endl;
            }
            //after output, delete other lines until all the ||iginal Camera msg is removed 
            findCamera = 3;
        }
    }
    infile.close();
    outfile.close();
    return newfilenames;
}

void ChangeLookAt(RealisticCameraParam RC,std::ifstream &infile,std::ofstream &outfile){
    if("" == RC.Look_at){
        return;
    }
    std::string temp;
    while (!infile.eof())
    {
        getline(infile,temp); 
        if("LookAt"==temp.substr(0,6)){
            outfile << RC.Look_at << std::endl;
            return;
        }
    }
    return;
}
void ChangeLookAt(OrthographicCameraParam RC,std::ifstream &infile,std::ofstream &outfile){
    if("" == RC.Look_at){
        return;
    }
    std::string temp;
    while (!infile.eof())
    {
        getline(infile,temp); 
        if("LookAt"==temp.substr(0,6)){
            outfile << RC.Look_at << std::endl;
            return;
        }
    }
    return;
}
void ChangeLookAt(SphericalCameraParam RC,std::ifstream &infile,std::ofstream &outfile){
    if("" == RC.Look_at){
        return;
    }
    std::string temp;
    while (!infile.eof())
    {
        getline(infile,temp); 
        if("LookAt"==temp.substr(0,6)){
            outfile << RC.Look_at << std::endl;
            return;
        }
    }
    return;
}
void ChangeLookAt(PerspectiveCameraParam RC,std::ifstream &infile,std::ofstream &outfile){
    if("" == RC.Look_at){
        return;
    }
    std::string temp;
    while (!infile.eof())
    {
        getline(infile,temp);
        if("LookAt"==temp.substr(0,6)){
            outfile << RC.Look_at << std::endl;
            return;
        }
    }
    return;
}