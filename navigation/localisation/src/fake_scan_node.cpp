#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>

#include "Point.h"

int main(int argc, char** argv)
{
    ROS_INFO("Starting node fake_scan_node");

    //Initialisation du noeud ROS
    ros::init(argc, argv, "fake_scan_node");
    
    ros::NodeHandle n;

    //on publie un scan laser sur le topic /fake_scan
    ros::Publisher pub_scan = n.advertise<sensor_msgs::LaserScan>("/fake_scan", 1000);

    //construction du faux scan
    sensor_msgs::LaserScanPtr scan(new sensor_msgs::LaserScan());

    scan->angle_min = -M_PI_2;
    scan->angle_max =  M_PI_2;
    scan->angle_increment = 0.00613592332229;
    scan->range_min = 0;
    scan->range_max = 5;

    std::vector<float> ranges = {2.3310000896453857, 2.322000026702881, 2.309999942779541, 2.2950000762939453, 2.2920000553131104, 2.2799999713897705, 2.2699999809265137, 2.265000104904175, 2.259999990463257, 2.256999969482422, 2.2290000915527344, 2.2269999980926514, 2.2260000705718994, 2.2260000705718994, 2.2039999961853027, 2.2009999752044678, 2.200000047683716, 2.186000108718872, 2.1730000972747803, 2.171999931335449, 2.1679999828338623, 2.1649999618530273, 2.1600000858306885, 2.1459999084472656, 2.138000011444092, 2.134000062942505, 2.127000093460083, 2.119999885559082, 2.115999937057495, 2.1110000610351562, 2.109999895095825, 2.0980000495910645, 2.0969998836517334, 2.0920000076293945, 2.0840001106262207, 2.0829999446868896, 2.0829999446868896, 2.0789999961853027, 2.0759999752044678, 2.055999994277954, 2.055999994277954, 2.055000066757202, 2.055999994277954, 2.055999994277954, 2.046999931335449, 2.0460000038146973, 2.0369999408721924, 2.0339999198913574, 2.0299999713897705, 2.0239999294281006, 2.0239999294281006, 2.0209999084472656, 2.0190000534057617, 2.0190000534057617, 2.0190000534057617, 2.010999917984009, 2.007999897003174, 2.003999948501587, 1.9950000047683716, 1.9930000305175781, 1.9930000305175781, 1.9930000305175781, 1.9900000095367432, 1.9880000352859497, 1.9880000352859497, 1.9869999885559082, 1.9869999885559082, 1.9819999933242798, 1.9789999723434448, 1.9739999771118164, 1.9620000123977661, 1.9609999656677246, 1.9609999656677246, 1.9600000381469727, 1.9600000381469727, 1.9600000381469727, 1.9579999446868896, 1.9570000171661377, 1.9570000171661377, 1.9570000171661377, 1.9509999752044678, 1.9509999752044678, 1.9600000381469727, 1.9600000381469727, 1.9600000381469727, 1.9600000381469727, 1.9559999704360962, 1.9529999494552612, 1.9470000267028809, 1.940999984741211, 1.940000057220459, 1.940000057220459, 1.940000057220459, 1.940000057220459, 1.9429999589920044, 1.9429999589920044, 1.9429999589920044, 1.9450000524520874, 1.9450000524520874, 1.9429999589920044, 1.9450000524520874, 1.9429999589920044, 1.9429999589920044, 1.9450000524520874, 1.9470000267028809, 1.9470000267028809, 1.9470000267028809, 1.9470000267028809, 1.9470000267028809, 1.9459999799728394, 1.9459999799728394, 1.944000005722046, 1.944000005722046, 1.9539999961853027, 1.9539999961853027, 1.9539999961853027, 1.9539999961853027, 1.9559999704360962, 1.9559999704360962, 1.9620000123977661, 1.9620000123977661, 1.9630000591278076, 1.965999960899353, 1.965999960899353, 1.9670000076293945, 1.9670000076293945, 1.9759999513626099, 1.9780000448226929, 1.9780000448226929, 1.9780000448226929, 1.9780000448226929, 1.9830000400543213, 1.9880000352859497, 1.9889999628067017, 1.9919999837875366, 1.9889999628067017, 1.996000051498413, 1.9980000257492065, 1.9989999532699585, 2.005000114440918, 2.015000104904175, 2.015000104904175, 2.0169999599456787, 2.0199999809265137, 2.0290000438690186, 2.0290000438690186, 2.0329999923706055, 2.0329999923706055, 2.0290000438690186, 0.5809999704360962, 0.578000009059906, 0.578000009059906, 0.5730000138282776, 0.5680000185966492, 0.5680000185966492, 0.5609999895095825, 0.5600000023841858, 0.5550000071525574, 0.550000011920929, 0.5490000247955322, 0.5440000295639038, 0.5400000214576721, 0.5400000214576721, 0.5400000214576721, 0.5370000004768372, 0.5299999713897705, 0.5299999713897705, 0.5299999713897705, 0.5270000100135803, 0.5230000019073486, 0.5230000019073486, 0.5220000147819519, 0.5210000276565552, 0.5199999809265137, 0.5189999938011169, 0.5170000195503235, 0.5170000195503235, 0.5070000290870667, 0.5019999742507935, 0.5019999742507935, 0.5019999742507935, 0.4970000088214874, 0.4970000088214874, 0.4959999918937683, 0.49399998784065247, 0.48899999260902405, 0.4880000054836273, 0.4880000054836273, 0.4880000054836273, 0.4880000054836273, 0.4880000054836273, 0.4880000054836273, 0.48500001430511475, 0.48500001430511475, 0.4779999852180481, 0.47699999809265137, 0.47600001096725464, 0.47600001096725464, 0.47600001096725464, 0.47600001096725464, 0.47600001096725464, 0.4729999899864197, 0.47200000286102295, 0.4650000035762787, 0.4650000035762787, 0.4650000035762787, 0.4650000035762787, 0.4650000035762787, 0.45899999141693115, 0.45899999141693115, 0.45399999618530273, 0.45399999618530273, 0.45399999618530273, 0.46399998664855957, 0.46399998664855957, 0.46399998664855957, 0.46000000834465027, 0.4620000123977661, 0.46000000834465027, 0.46000000834465027, 0.46000000834465027, 0.46000000834465027, 0.46000000834465027, 0.46000000834465027, 0.4580000042915344, 0.4580000042915344, 0.4580000042915344, 0.45399999618530273, 0.45100000500679016, 0.44999998807907104, 0.44699999690055847, 0.44699999690055847, 0.44699999690055847, 0.44699999690055847, 0.44699999690055847, 0.4480000138282776, 0.4480000138282776, 0.4490000009536743, 0.4490000009536743, 0.4480000138282776, 0.44600000977516174, 0.4490000009536743, 0.4490000009536743, 0.44200000166893005, 0.4490000009536743, 0.4490000009536743, 0.4480000138282776, 0.4480000138282776, 0.4480000138282776, 0.4480000138282776, 0.4410000145435333, 0.4410000145435333, 0.4410000145435333, 0.4410000145435333, 0.44600000977516174, 0.4480000138282776, 0.44999998807907104, 0.44999998807907104, 0.44999998807907104, 0.453000009059906, 0.45899999141693115, 0.453000009059906, 0.453000009059906, 0.453000009059906, 0.44999998807907104, 0.44999998807907104, 0.46000000834465027, 0.46399998664855957, 0.46399998664855957, 0.46399998664855957, 0.4620000123977661, 0.4620000123977661, 0.44999998807907104, 0.44999998807907104, 0.44999998807907104, 0.44999998807907104, 0.44999998807907104, 0.4490000009536743, 0.4490000009536743, 0.4480000138282776, 0.4480000138282776, 0.44699999690055847, 0.44699999690055847, 0.44600000977516174, 0.4449999928474426, 0.4449999928474426, 0.44600000977516174, 0.4480000138282776, 0.4490000009536743, 0.4519999921321869, 0.4519999921321869, 0.4490000009536743, 0.44699999690055847, 0.44699999690055847, 0.44200000166893005, 0.4399999976158142, 0.4399999976158142, 0.4440000057220459, 0.4399999976158142, 0.4440000057220459, 0.45100000500679016, 0.45100000500679016, 0.45100000500679016, 0.45100000500679016, 0.44999998807907104, 0.4480000138282776, 0.4480000138282776, 0.4480000138282776, 0.4480000138282776, 0.4480000138282776, 0.4480000138282776, 0.4480000138282776, 0.4490000009536743, 0.4490000009536743, 0.453000009059906, 0.453000009059906, 0.4569999873638153, 0.453000009059906, 0.45399999618530273, 0.45399999618530273, 0.45399999618530273, 0.45399999618530273, 0.4580000042915344, 0.45899999141693115, 0.4659999907016754, 0.4659999907016754, 0.4650000035762787, 0.46299999952316284, 0.4650000035762787, 0.4650000035762787, 0.4690000116825104, 0.47099998593330383, 0.47099998593330383, 0.47099998593330383, 0.47099998593330383, 0.47099998593330383, 0.47099998593330383, 0.4749999940395355, 0.4790000021457672, 0.4819999933242798, 0.4819999933242798, 0.48500001430511475, 0.48500001430511475, 0.48500001430511475, 0.48500001430511475, 0.48399999737739563, 0.48399999737739563, 0.48899999260902405, 0.4950000047683716, 0.49000000953674316, 0.4950000047683716, 0.4950000047683716, 0.49799999594688416, 0.5019999742507935, 0.503000020980835, 0.5049999952316284, 0.5049999952316284, 0.5059999823570251, 0.5059999823570251, 0.5059999823570251, 0.5070000290870667, 0.5080000162124634, 0.5090000033378601, 0.5120000243186951, 0.5149999856948853, 0.5210000276565552, 0.5230000019073486, 0.5230000019073486, 0.5230000019073486, 0.5270000100135803, 0.5320000052452087, 0.5220000147819519, 0.5320000052452087, 0.5320000052452087, 1.9049999713897705, 1.906000018119812, 1.909000039100647, 1.909000039100647, 1.909000039100647, 1.909000039100647, 1.9129999876022339, 1.9129999876022339, 1.9129999876022339, 1.9110000133514404, 1.9129999876022339, 1.9119999408721924, 1.9119999408721924, 1.9170000553131104, 1.9240000247955322, 1.9240000247955322, 1.9320000410079956, 1.9320000410079956, 1.9359999895095825, 1.9429999589920044, 1.9470000267028809, 1.9470000267028809, 1.9470000267028809, 1.9479999542236328, 1.9550000429153442, 1.9550000429153442, 1.9550000429153442, 1.9600000381469727, 1.9730000495910645, 1.975000023841858, 1.9819999933242798, 1.9819999933242798, 1.99399995803833, 1.9950000047683716, 2.000999927520752, 2.005000114440918, 2.00600004196167, 2.007999897003174, 2.0160000324249268, 2.0160000324249268, 2.0269999504089355, 2.0280001163482666, 2.0429999828338623, 2.0450000762939453, 2.046999931335449, 2.046999931335449, 2.072000026702881, 2.072999954223633, 2.072999954223633, 2.0739998817443848, 2.075000047683716, 2.0850000381469727, 2.0980000495910645, 2.0999999046325684, 2.1050000190734863, 2.121000051498413, 2.122999906539917, 2.122999906539917, 2.1389999389648438, 2.1440000534057617, 2.1570000648498535, 2.1579999923706055, 2.1700000762939453, 2.177999973297119, 2.184999942779541, 2.188999891281128, 2.196000099182129, 2.196000099182129, 2.188999891281128, 2.184000015258789, 2.1570000648498535, 2.135999917984009, 2.0989999771118164, 2.0840001106262207, 2.059000015258789, 2.052000045776367, 2.0239999294281006, 1.996000051498413, 1.9919999837875366, 1.9759999513626099, 1.9550000429153442, 1.9509999752044678, 1.9149999618530273, 1.909000039100647, 1.8980000019073486, 1.8860000371932983, 1.8830000162124634, 1.8569999933242798, 1.8450000286102295, 1.8240000009536743, 1.815000057220459, 1.7990000247955322, 1.7869999408721924, 1.7740000486373901, 1.753000020980835, 1.746000051498413, 1.7289999723434448, 1.7289999723434448, 1.7050000429153442, 1.7000000476837158, 1.680999994277954, 1.6629999876022339, 1.6579999923706055, 1.6579999923706055, 1.6399999856948853, 1.6239999532699585, 1.6160000562667847, 1.5980000495910645, 1.590000033378601, 1.5870000123977661, 1.5679999589920044, 1.5579999685287476, 1.5520000457763672, 1.5440000295639038, 1.534999966621399, 1.5329999923706055, 1.5290000438690186, 1.5180000066757202, 1.4980000257492065, 1.4930000305175781, 1.4880000352859497, 1.4819999933242798, 1.475000023841858, 1.4670000076293945, 1.4639999866485596, 1.4490000009536743, 1.4420000314712524, 1.4420000314712524, 1.4490000009536743, 1.4600000381469727, 1.5230000019073486, 2.7699999809265137, 2.7699999809265137, 2.7699999809265137, 2.74399995803833, 2.74399995803833, 2.740000009536743, 2.740000009536743, 2.7249999046325684};
    
    scan->ranges = ranges;
    
    ros::Rate loop_rate (10);

    while(n.ok())
    {
        // Publish
        pub_scan.publish(scan);

        // Spin
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}