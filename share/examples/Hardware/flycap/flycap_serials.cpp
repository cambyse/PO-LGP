#include <Hardware/flycap/flycap.h>
#include <iostream>

using namespace std;
using namespace std::placeholders;
using namespace MLR;

int main(int argc, char* argv[])
{
	try {
		vector<uint32_t> serials(get_flycap_ids());
		if(serials.size() == 0) {
			cout << "No cameras found." << endl;
		} else {
			cout << serials.size() << " cameras found." << endl;
		}
		for(int i = 0, len = serials.size(); i < len; ++i) {
			cout << "Flycap camera " << i << " has serial " << serials[i] << endl;
		}
	
	} catch(const std::exception& ex) {
		cerr << ex.what() << endl;
	}

	return 0;
}

