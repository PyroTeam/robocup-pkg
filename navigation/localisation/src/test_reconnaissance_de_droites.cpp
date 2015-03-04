#include "ros/ros.h"
#include "deplacement_msg/Points.h"
#include "deplacement_msg/Droites.h"
#include "sensor_msgs/LaserScan.h"
#include "line_detection_utils.h"
#include <ros/console.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include "laserScan.h"
#include <initializer_list>

int main(int argc, char** argv)
{
    laserScan laserData;

	sensor_msgs::LaserScanPtr scan(new sensor_msgs::LaserScan());
	sensor_msgs::LaserScanConstPtr pscan(scan);

	scan->angle_min = -M_PI_2;
	scan->angle_max =  M_PI_2;
	scan->angle_increment = 0.00613592332229;
	scan->range_min = 0;
	scan->range_max = 5;

	std::vector<float> ranges = {0.23800000548362732, 0.2370000034570694, 0.23800000548362732, 0.23800000548362732, 0.24199999868869781, 0.23800000548362732, 0.23999999463558197, 0.23800000548362732, 0.23999999463558197, 0.23999999463558197, 0.24300000071525574, 0.24400000274181366, 0.24400000274181366, 0.24400000274181366, 0.23999999463558197, 0.23800000548362732, 0.23800000548362732, 0.23999999463558197, 0.24199999868869781, 0.24400000274181366, 0.24400000274181366, 0.24799999594688416, 0.24799999594688416, 0.24799999594688416, 0.24799999594688416, 0.24799999594688416, 0.24799999594688416, 0.24799999594688416, 0.24899999797344208, 0.24799999594688416, 0.24899999797344208, 0.24899999797344208, 0.24899999797344208, 0.2460000067949295, 0.2409999966621399, 0.23800000548362732, 0.23800000548362732, 0.2409999966621399, 0.2460000067949295, 0.25, 0.25099998712539673, 0.25099998712539673, 0.2540000081062317, 0.2540000081062317, 0.2460000067949295, 0.2460000067949295, 0.25, 0.2460000067949295, 0.25, 0.25, 0.2460000067949295, 0.24400000274181366, 0.2460000067949295, 0.2460000067949295, 0.24699999392032623, 0.2529999911785126, 0.2540000081062317, 0.25699999928474426, 0.26600000262260437, 0.25699999928474426, 0.2540000081062317, 0.2529999911785126, 0.2529999911785126, 0.2529999911785126, 0.2680000066757202, 0.2680000066757202, 0.2680000066757202, 0.2680000066757202, 0.2680000066757202, 0.2639999985694885, 0.26899999380111694, 0.27000001072883606, 0.26899999380111694, 0.2630000114440918, 0.27000001072883606, 0.26899999380111694, 0.26899999380111694, 0.26899999380111694, 0.26899999380111694, 0.26899999380111694, 0.26899999380111694, 0.2630000114440918, 0.2630000114440918, 0.26899999380111694, 0.2619999945163727, 0.26899999380111694, 0.26899999380111694, 0.27300000190734863, 0.289000004529953, 0.289000004529953, 0.2849999964237213, 0.2849999964237213, 0.27900001406669617, 0.27900001406669617, 0.2849999964237213, 0.28600001335144043, 0.2930000126361847, 0.2930000126361847, 0.2980000078678131, 0.30000001192092896, 0.30000001192092896, 0.3009999990463257, 0.3009999990463257, 0.3009999990463257, 0.30000001192092896, 0.3009999990463257, 0.3019999861717224, 0.3050000071525574, 0.3050000071525574, 0.3050000071525574, 0.31299999356269836, 0.3190000057220459, 0.3230000138282776, 0.3190000057220459, 0.32199999690055847, 0.32199999690055847, 0.32199999690055847, 0.32199999690055847, 0.3230000138282776, 0.3230000138282776, 0.32499998807907104, 0.328000009059906, 0.3370000123977661, 0.3370000123977661, 0.3370000123977661, 0.3440000116825104, 0.3440000116825104, 0.34599998593330383, 0.34299999475479126, 0.34599998593330383, 0.34700000286102295, 0.3540000021457672, 0.3540000021457672, 0.35600000619888306, 0.36399999260902405, 0.3619999885559082, 0.36399999260902405, 0.37299999594688416, 0.38100001215934753, 0.38100001215934753, 0.38100001215934753, 0.38100001215934753, 0.38100001215934753, 0.38199999928474426, 0.38600000739097595, 0.38600000739097595, 0.39899998903274536, 0.40299999713897705, 0.40299999713897705, 0.40299999713897705, 0.40299999713897705, 0.40299999713897705, 0.40700000524520874, 0.4259999990463257, 0.4300000071525574, 0.4339999854564667, 0.43799999356269836, 0.4390000104904175, 0.4399999976158142, 0.4410000145435333, 0.4480000138282776, 0.45899999141693115, 0.46000000834465027, 0.4620000123977661, 0.46399998664855957, 0.46700000762939453, 0.47600001096725464, 0.4909999966621399, 0.49300000071525574, 0.49799999594688416, 0.5059999823570251, 0.5090000033378601, 0.5149999856948853, 0.5230000019073486, 0.5299999713897705, 0.531000018119812, 0.5370000004768372, 0.5419999957084656, 0.5559999942779541, 0.5640000104904175, 0.5699999928474426, 0.5759999752044678, 0.5849999785423279, 0.5870000123977661, 0.5929999947547913, 0.5979999899864197, 0.6159999966621399, 0.6179999709129333, 0.628000020980835, 0.6489999890327454, 0.6570000052452087, 0.671999990940094, 0.6840000152587891, 0.6949999928474426, 0.7170000076293945, 0.7250000238418579, 0.7350000143051147, 0.7369999885559082, 0.753000020980835, 0.7710000276565552, 0.7850000262260437, 0.8050000071525574, 0.8090000152587891, 0.8199999928474426, 0.8519999980926514, 0.8669999837875366, 0.8790000081062317, 0.8949999809265137, 0.9210000038146973, 0.9319999814033508, 0.9490000009536743, 0.9810000061988831, 1.0140000581741333, 1.0399999618530273, 1.065000057220459, 1.0850000381469727, 1.1260000467300415, 1.152999997138977, 1.1670000553131104, 1.215000033378601, 1.2599999904632568, 1.3020000457763672, 1.3359999656677246, 1.3910000324249268, 1.4479999542236328, 1.5049999952316284, 1.562999963760376, 1.625, 1.7059999704360962, 1.7879999876022339, 1.8609999418258667, 1.940000057220459, 2.055999994277954, 2.1760001182556152, 2.2720000743865967, 2.4119999408721924, 2.6010000705718994, 2.7690000534057617, 2.7690000534057617, 2.9240000247955322, 3.121999979019165, 3.6610000133514404, 4.1620001792907715, 5.599999904632568, 4.255000114440918, 4.255000114440918, 4.255000114440918, 4.250999927520752, 4.251999855041504, 4.251999855041504, 4.250999927520752, 4.251999855041504, 4.251999855041504, 4.255000114440918, 4.255000114440918, 4.265999794006348, 4.265999794006348, 4.265999794006348, 4.260000228881836, 4.25600004196167, 4.25600004196167, 4.255000114440918, 4.255000114440918, 4.255000114440918, 4.255000114440918, 4.255000114440918, 4.265999794006348, 4.267000198364258, 4.2729997634887695, 4.2729997634887695, 4.275000095367432, 4.276000022888184, 4.284999847412109, 4.285999774932861, 4.2870001792907715, 4.289999961853027, 4.300000190734863, 4.309000015258789, 4.309000015258789, 4.309999942779541, 4.314000129699707, 4.315999984741211, 4.317999839782715, 4.336999893188477, 4.3379998207092285, 4.3420000076293945, 4.3470001220703125, 4.36299991607666, 4.36299991607666, 4.367000102996826, 4.368000030517578, 4.386000156402588, 4.389999866485596, 4.390999794006348, 4.39900016784668, 4.418000221252441, 4.418000221252441, 4.418000221252441, 4.431000232696533, 4.431000232696533, 4.431000232696533, 2.609999895095825, 2.5260000228881836, 2.502000093460083, 2.502000093460083, 2.490999937057495, 2.438999891281128, 2.433000087738037, 2.431999921798706, 2.431999921798706, 2.433000087738037, 2.444000005722046, 2.447000026702881, 2.4539999961853027, 2.4600000381469727, 2.4600000381469727, 2.4660000801086426, 2.4749999046325684, 2.490999937057495, 2.492000102996826, 4.682000160217285, 4.683000087738037, 4.685999870300293, 4.693999767303467, 4.7230000495910645, 4.724999904632568, 4.742000102996826, 4.757999897003174, 4.7769999504089355, 4.7820000648498535, 4.807000160217285, 4.809000015258789, 4.848999977111816, 4.849999904632568, 4.86299991607666, 4.880000114440918, 4.8979997634887695, 4.9019999504089355, 4.932000160217285, 4.933000087738037, 4.949999809265137, 4.9670000076293945, 4.985000133514404, 5.014999866485596, 5.03000020980835, 5.041999816894531, 5.059999942779541, 5.085999965667725, 5.11299991607666, 5.13100004196167, 5.13100004196167, 5.13100004196167, 3.6670000553131104, 3.6480000019073486, 3.6089999675750732, 3.5869998931884766, 3.555000066757202, 3.5230000019073486, 3.49399995803833, 3.4730000495910645, 3.4539999961853027, 3.4159998893737793, 3.3940000534057617, 3.367000102996826, 3.365999937057495, 3.309000015258789, 3.302000045776367, 3.2860000133514404, 3.2860000133514404, 3.2860000133514404, 3.308000087738037, 3.315999984741211, 3.3239998817443848, 3.3239998817443848, 3.3489999771118164, 3.368000030517578, 3.4010000228881836, 3.4240000247955322, 3.428999900817871, 3.443000078201294, 3.4639999866485596, 3.503999948501587, 3.513000011444092, 3.5399999618530273, 3.572000026702881, 3.6059999465942383, 3.615999937057495, 5.002999782562256, 5.002999782562256, 4.98799991607666, 4.98799991607666, 4.966000080108643, 4.928999900817871, 4.908999919891357, 4.885000228881836, 4.859000205993652, 2.138000011444092, 2.125999927520752, 2.125, 2.1040000915527344, 2.1019999980926514, 2.0989999771118164, 2.0880000591278076, 2.063999891281128, 2.059000015258789, 2.052000045776367, 2.0510001182556152, 2.0429999828338623, 2.0339999198913574, 2.0239999294281006, 2.0160000324249268, 2.010999917984009, 2.007999897003174, 2.0, 2.0, 2.0, 2.010999917984009, 2.0380001068115234, 4.375999927520752, 4.375999927520752, 4.375999927520752, 4.341000080108643, 4.328000068664551, 4.315999984741211, 4.304999828338623, 4.26200008392334, 4.25600004196167, 4.25600004196167, 4.243000030517578, 4.232999801635742, 4.201000213623047, 4.198999881744385, 4.177000045776367, 4.168000221252441, 4.146999835968018, 4.139999866485596, 4.135000228881836, 4.114999771118164, 4.105999946594238, 4.091000080108643, 4.084000110626221, 4.080999851226807, 4.061999797821045, 4.052000045776367, 4.039999961853027, 4.0370001792907715, 4.011000156402588, 4.00600004196167, 3.996000051498413, 3.9830000400543213, 3.9790000915527344, 3.9649999141693115, 3.9539999961853027, 3.940000057220459, 3.938999891281128, 3.937000036239624, 3.927999973297119, 3.9210000038146973, 3.9179999828338623, 3.9040000438690186, 3.9019999504089355, 3.9000000953674316, 3.8959999084472656, 3.880000114440918, 3.878999948501587, 3.875, 3.875, 3.868000030517578, 3.8610000610351562, 3.8570001125335693, 3.8499999046325684, 3.8389999866485596, 3.8329999446868896, 3.8310000896453857, 3.8269999027252197, 3.8239998817443848, 3.822000026702881, 3.819999933242798, 3.811000108718872, 3.806999921798706, 3.8010001182556152, 3.8010001182556152, 3.7950000762939453, 3.7950000762939453, 3.7939999103546143, 3.7880001068115234, 3.7839999198913574, 3.7809998989105225, 3.7780001163482666, 3.7780001163482666, 3.7780001163482666, 3.7750000953674316, 3.7669999599456787, 3.7669999599456787, 3.7660000324249268, 3.757999897003174, 3.757999897003174, 3.757999897003174, 3.755000114440918, 3.755000114440918, 3.753000020980835, 3.753000020980835, 3.752000093460083, 3.746999979019165, 3.744999885559082, 3.744999885559082, 3.746999979019165, 3.744999885559082, 3.753999948501587, 3.757999897003174, 3.757999897003174, 3.757999897003174, 3.757999897003174};
	scan->ranges = ranges;

    laserData.setRanges(pscan);

    std::list<Point>  listOfPoints = laserData.getPoints();
    std::list<Modele> listOfModeles = findLines(listOfPoints);

    for(auto &it : listOfModeles){
        std::cout << "m = " << it.getDroite().getPente() << " p = " << it.getDroite().getOrdOrigin() << std::endl;
    	std::cout << "nb de points dans la droite : " << it.getIndex().size() << std::endl;
        std::cout << "coeff de correlation : " << it.getCorrel() << std::endl;
    }

    return 0;
}
