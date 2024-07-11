//
// lenstool.cpp
//
// Various useful operations on creating lens file in json format.
//
#include <pbrt/pbrt.h>

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <string>
#include <fstream>
#include <iomanip>
#include <pbrt/options.h>
#include <pbrt/util/error.h>
#include <pbrt/util/args.h>
#include <pbrt/util/file.h>
#include <pbrt/util/image.h>
#include <pbrt/util/spectrum.h>
#include <pbrt/util/parallel.h>
#include <pbrt/util/transform.h>
#include <ext/json.hpp>
#include <pbrt/util/log.h>
using namespace nlohmann;
using namespace pbrt;

static void usage(const char *msg = nullptr, ...) {
    if (msg) {
        va_list args;
        va_start(args, msg);
        fprintf(stderr, "lenstool: ");
        vfprintf(stderr, msg, args);
        fprintf(stderr, "\n");
    }
    fprintf(stderr, R"(usage: lenstool <command> [options] <filenames...>

commands: convert insertmicrolens

convert options:
    --inputscale <n>    Input units per mm (which are used in the output). Default: 1.0
    --implicitdefaults  Omit fields from the json file if they match the defaults.

insertmicrolens options:
    --xdim <n>             How many microlenses span the X direction. Default: 16
    --ydim <n>             How many microlenses span the Y direction. Default: 16
    --filmwidth <n>        Width of target film in mm. Default: 20.0
    --filmheight <n>       Height of target film in mm. Default: 20.0
    --filmtolens <n>       Distance from film to back of main lens system (in mm). Default: 50.0
)");
    exit(1);
}

namespace pbrt {
    void to_json(json& j, const Transform& t) {
        SquareMatrix<4> m = t.GetMatrix();
        for (int c = 0; c < 4; ++c) {
            json col;
            for (int r = 0; r < 4; ++r) {
                col.push_back(m[c][r]);
            }
            j.push_back(col);
        }
    }
}

static bool endsWith(const std::string& str, const std::string& suffix) {
    return str.size() >= suffix.size() && 0 == str.compare(str.size() - suffix.size(), suffix.size(), suffix);
};

int convert(int argc, char *argv[]) {
    float scale = 1.f;
    bool implicitDefaults = false;
    int i;
    auto parseArg = [&]() -> std::pair<std::string, double> {
        const char *ptr = argv[i];
        // Skip over a leading dash or two.
        CHECK_EQ(*ptr, '-');
        ++ptr;
        if (*ptr == '-') ++ptr;

        // Copy the flag name to the string.
        std::string flag;
        while (*ptr && *ptr != '=') flag += *ptr++;

        if (!*ptr && i + 1 == argc)
            usage("missing value after %s flag", argv[i]);
        const char *value = (*ptr == '=') ? (ptr + 1) : argv[++i];
        return {flag, atof(value)};
    };

    std::pair<std::string, double> arg;
    for (i = 0; i < argc; ++i) {
        if (argv[i][0] != '-') break;
        if (!strcmp(argv[i], "--implicitdefaults") || !strcmp(argv[i], "-implicitdefaults"))
            implicitDefaults = !implicitDefaults;
        else {
            std::pair<std::string, double> arg = parseArg();
            if (std::get<0>(arg) == "inputscale") {
                scale = std::get<1>(arg);
                if (scale == 0) usage("--inputscale value must be non-zero");
            }
            else
                usage();
        }
    }

    if (i >= argc)
        usage("missing filenames for \"convert\""); 
    else if (i + 1 >= argc)
        usage("missing second filename (output) for \"convert\"");

    const char *inFilename = argv[i], *outFilename = argv[i + 1];

    json identity; 
    pbrt::to_json(identity, Transform(SquareMatrix<4>()));
    json jwavelengths;
    for (int i = 0; i < NSpectrumSamples; ++i) {
        Float t = ((Float)i)/(NSpectrumSamples-1);
        Float lambda = (1.0 - t)*Lambda_min + t*Lambda_max;
        jwavelengths.push_back(lambda);
    }

    std::string lensFile = inFilename;
    if (endsWith(lensFile, ".dat")) {
        // Load element data from lens description file
        std::vector<Float> lensData = ReadFloatFile(lensFile);
        std::string name;
        std::string description;
        if (lensData.empty()) {
            Error("Error reading lens specification file \"%s\".",
                lensFile);
            return -1;
        } else {
            // TODO: More robust extraction of names, descriptions
            std::ifstream infile(lensFile);
            if (infile.good()) {
                std::string nameLine, descriptionLine;
                getline(infile, nameLine);
                if (nameLine[0] == '#') {
                    name = nameLine.substr(1);
                }
                getline(infile, descriptionLine);
                while (descriptionLine[0] == '#') {
                    description += descriptionLine.substr(1);
                    getline(infile, descriptionLine);
                } 
            }
        }
        if (lensData.size() % 4 != 0) {
            // Trisha: If the size has an extra value, it's possible this lens type was meant for pbrt-v2-spectral and has an extra focal length value at the top. In this case, let's automatically convert it by removing this extra value.
            if (lensData.size() % 4 == 1) {
                Warning("Extra value in lens specification file, this lens file may be for pbrt-v2-spectral. Removing extra value to make it compatible with pbrt-v3-spectral...");
                lensData.erase(lensData.begin());
            }
            else {
                Error(
                    "Excess values in lens specification file \"%s\"; "
                    "must be multiple-of-four values, read %d.",
                    lensFile.c_str(), (int)lensData.size());
                return -1;
            }
        }
        
        std::vector<json> surfaces;
        for (int i = 0; i < lensData.size(); i += 4) {
            json jsurf;
            jsurf["radius"] = lensData[i+0];
            jsurf["thickness"] = lensData[i+1];
            jsurf["semi_aperture"] = lensData[i+3] / 2.0f;
            Float ior = lensData[i + 2];
            if (implicitDefaults) {
                jsurf["ior"] = ior;
            } else {
                json jior;
                for (int i = 0; i < NSpectrumSamples; ++i) {
                    jior.push_back(ior);
                }
                jsurf["ior"] = {jwavelengths,jior};
                jsurf["conic_constant"] = (Float)0.0;
                jsurf["transform"] = identity;
            }
            surfaces.push_back(jsurf);
        }

        json j;
        j["name"] = name;
        j["description"] = description;
        j["surfaces"] = surfaces;
        std::ofstream outfile(outFilename);
        outfile << std::setw(4) << j << std::endl;

        // Covert to outfile here
        printf("Input file: %s, Ouput file: %s; %zd surfaces\n", inFilename, outFilename, surfaces.size());
        
    } else {
        Error(
            "Input to lenstool conver must be a .dat file, but given \"%s\"; ",
            lensFile.c_str());
    }

    return 0;
}

int insertmicrolens(int argc, char *argv[]) {
    int xDim = 16;
    int yDim = 16;
    Float filmWidth = 20.0;
    Float filmHeight = 20.0;
    Float filmToLens = 50.0;
    Float filmToMicrolens = 0.0;
    int i;
    auto parseArg = [&]() -> std::pair<std::string, double> {
        const char *ptr = argv[i];
        // Skip over a leading dash or two.
        CHECK_EQ(*ptr, '-');
        ++ptr;
        if (*ptr == '-') ++ptr;

        // Copy the flag name to the string.
        std::string flag;
        while (*ptr && *ptr != '=') flag += *ptr++;

        if (!*ptr && i + 1 == argc)
            usage("missing value after %s flag", argv[i]);
        const char *value = (*ptr == '=') ? (ptr + 1) : argv[++i];
        return{ flag, atof(value) };
    }; 

    std::pair<std::string, double> arg;
    for (i = 0; i < argc; ++i) {
        if (argv[i][0] != '-') break;
        std::pair<std::string, double> arg = parseArg();
        if (std::get<0>(arg) == "xdim") {
            xDim = (int)std::get<1>(arg);
            if (xDim <= 0) usage("--xDim value must be positive");
        } 
        else if (std::get<0>(arg) == "ydim") {
            yDim = (int)std::get<1>(arg);
            if (yDim <= 0) usage("--yDim value must be positive");
        }
        else if (std::get<0>(arg) == "filmwidth") {
            filmWidth = (Float)std::get<1>(arg);
            if (filmWidth <= 0) usage("--filmwidth value must be positive");
        }
        else if (std::get<0>(arg) == "filmheight") {
            filmHeight = (Float)std::get<1>(arg);
            if (filmHeight <= 0) usage("--filmheight value must be positive");
        }
        else if (std::get<0>(arg) == "filmtolens") {
            filmToLens = (Float)std::get<1>(arg);
            if (filmToLens <= 0) usage("--filmtolens value must be positive");
        }
        else if (std::get<0>(arg) == "filmtomicrolens") {
            filmToMicrolens = (Float)std::get<1>(arg);
            if (filmToMicrolens != 0.0) usage("--filmtomicrolens currently must be exactly 0! More complicated algorithms NYI.");
        }
        else
            usage();
    }

    if (i >= argc)
        usage("missing filenames for \"insertmicrolens\"");
    else if (i + 1 >= argc)
        usage("missing second filename (microlens file) for \"insertmicrolens\"");
    else if (i + 2 >= argc)
        usage("missing third filename (output file) for \"insertmicrolens\"");

    const char *inFilename = argv[i], *microlensFilename = argv[i + 1], *outFilename = argv[i + 2];


    auto readJSONLenses = [](std::string lensFile, json& output) {
        if (!endsWith(lensFile, ".json")) {
            Error("Invalid format for lens specification file \"%s\".",
                lensFile.c_str());
            return false;
        }
        // read a JSON file
        std::ifstream i(lensFile);
        if (i && (i >> output)) {
            return true;
        } else {
            Error("Error reading lens specification file \"%s\".",
                lensFile.c_str());
            return false;
        }
    };
    json jlens, jmicrolens;
    readJSONLenses(inFilename, jlens);
    readJSONLenses(microlensFilename, jmicrolens);
    if (!jmicrolens["microlens"].is_null()) {
        Error("Error, microlens surface specification had its own microlens specification: \"%s\".",
            microlensFilename);
    }

    std::string mlname = jmicrolens["name"].get<std::string>();
    jlens["name"] = jlens["name"].get<std::string>() + " w/ microlens " + mlname;
    
    std::string microDescript = jmicrolens["description"].get<std::string>();
    if (microDescript == std::string("")) {
        microDescript = "\nWith added microlens " + mlname;
    } else {
        microDescript = "\nWith added microlens " + mlname + ":\n" + microDescript;
    }
    
    json outMicro;
    outMicro["dimensions"] = { xDim, yDim };
    outMicro["surfaces"] = jmicrolens["surfaces"];
    json offsets;
    for (int y = 0; y < yDim; ++y) {
        for (int x = 0; x < xDim; ++x) {
            offsets.push_back(json{ 0.0, 0.0 });
        }
    }
    outMicro["offsets"] = offsets;
    
    jlens["microlens"] = outMicro;
    std::ofstream outfile(outFilename);
    outfile << std::setw(4) << jlens << std::endl;

    // Covert to outfile here
    printf("%s + %s = %s\n", inFilename, microlensFilename, outFilename);

    return 0;
}

int main(int argc, char *argv[]) {
    PBRTOptions opt;
    InitPBRT(opt); // Warning and above.

    if (argc < 2) usage();

    if (!strcmp(argv[1], "convert"))
        return convert(argc - 2, argv + 2);
    else if (!strcmp(argv[1], "insertmicrolens"))
        return insertmicrolens(argc - 2, argv + 2);
    else
        usage("unknown command \"%s\"", argv[1]);
    /*
    std::vector<std::string> args = GetCommandLineArguments(argv);
    std::string cmd = args[0];
    args.erase(args.begin());
    if (cmd == "convert")
        return convert(args);
    else if (cmd == "insertmicrolens")
        return insertmicrolens(args);
    else
        fprintf(stderr, "lenstool: unknown command \"%s\".\n", cmd.c_str());
    */
    return 0;
}
