
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <libconfig.h++>

using namespace std;
using namespace libconfig;

// This example constructs a new configuration in memory and writes it to
// 'newconfig.cfg'.

int main(int argc, char **argv)
{
    static const char *output_file = "test.cfg";
    Config cfg;

    Setting &root = cfg.getRoot();

    // Add some settings to the configuration.
    //root
    Setting &preprocess = root.add("Preprocess", Setting::TypeGroup);
    Setting &segmentation = root.add("Segmentation", Setting::TypeGroup);
    Setting &keypoint = root.add("Keypoint", Setting::TypeGroup);
    Setting &matching = root.add("Matching", Setting::TypeGroup);
    Setting &recognition = root.add("Recognition", Setting::TypeGroup);


    // Preprocessor/Enalbe
    Setting &prep_enable = preprocess.add("Enable", Setting::TypeGroup);
    prep_enable.add("DoVoxel", Setting::TypeBoolean) = true;
    prep_enable.add("DoZFilter", Setting::TypeBoolean) = true;
    prep_enable.add("DoBilateral", Setting::TypeBoolean) = false;
    prep_enable.add("DoStatistic", Setting::TypeBoolean) = true;
    prep_enable.add("DoMlsSmooth", Setting::TypeBoolean) = false;
    prep_enable.add("DoRansac", Setting::TypeBoolean) = false;

    // Preprocessor/Voxel
    Setting &prep_voxel = preprocess.add("Voxel", Setting::TypeGroup);
    prep_voxel.add("sizeXYZ",Setting::TypeFloat) = 0.005f;
    // Preprocessor/ZFilter
    Setting &prep_zFilter = preprocess.add("ZFilter", Setting::TypeGroup);
    prep_zFilter.add("min",Setting::TypeFloat) = 0.5f;
    prep_zFilter.add("max",Setting::TypeFloat) = 2.0f;

    // Setting &prep_bilateral = preprocess.add("Bilateral", Setting::TypeGroup);
    // prep_bilateral.add("min",Setting::TypeFloat) = 0.5f;
    // prep_bilateral.add("max",Setting::TypeFloat) = 2.0f;

    Setting &prep_statistic = preprocess.add("Statistic", Setting::TypeGroup);
    prep_statistic.add("knnForMean",Setting::TypeInt) = 0.5f;
    prep_statistic.add("sigma",Setting::TypeFloat) = 2.0f;


    // Setting &prep_mlsSmooth = preprocess.add("mslSmooth", Setting::TypeGroup);
    // prep_mlsSmooth.add("min",Setting::TypeFloat) = 0.5f;
    // prep_mlsSmooth.add("max",Setting::TypeFloat) = 2.0f;
    
    tree.add("TreeType", Setting::TypeString) = "KdTreeFLANN";
    Setting &tree_kdtree = tree.add("", Setting::Type);

    address.add("city", Setting::TypeString) = "San Jose";
    address.add("state", Setting::TypeString) = "CA";
    address.add("zip", Setting::TypeInt) = 95110;

    Setting &array = root.add("array", Setting::TypeArray);

    for(int i = 0; i < 10; ++i)
        array.add(Setting::TypeInt) = 10 * i;

    // Write out the new configuration.
    try
    {
        cfg.writeFile(output_file);
        cerr << "New configuration successfully written to: " << output_file
            << endl;

    }
    catch(const FileIOException &fioex)
    {
        cerr << "I/O error while writing file: " << output_file << endl;
        return(EXIT_FAILURE);
    }

    return(EXIT_SUCCESS);
}
