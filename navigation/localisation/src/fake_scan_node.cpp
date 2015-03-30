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

    std::vector<float> ranges = {0.3970000147819519, 0.3970000147819519, 0.39800000190734863, 0.3970000147819519, 0.39399999380111694, 0.3840000033378601, 0.382999986410141, 0.382999986410141, 0.3840000033378601, 0.3869999945163727, 0.3889999985694885, 0.3889999985694885, 0.39100000262260437, 0.39100000262260437, 0.38600000739097595, 0.38600000739097595, 0.38600000739097595, 0.38600000739097595, 0.38999998569488525, 0.38999998569488525, 0.3970000147819519, 0.3970000147819519, 0.39800000190734863, 0.39800000190734863, 0.39800000190734863, 0.39500001072883606, 0.39800000190734863, 0.39800000190734863, 0.39800000190734863, 0.39800000190734863, 0.40299999713897705, 0.39800000190734863, 0.3959999978542328, 0.39500001072883606, 0.3959999978542328, 0.39500001072883606, 0.3959999978542328, 0.39500001072883606, 0.4020000100135803, 0.39500001072883606, 0.39399999380111694, 0.39399999380111694, 0.39399999380111694, 0.39399999380111694, 0.40400001406669617, 0.40700000524520874, 0.40400001406669617, 0.40700000524520874, 0.40400001406669617, 0.4009999930858612, 0.4009999930858612, 0.41200000047683716, 0.4009999930858612, 0.41200000047683716, 0.414000004529953, 0.41600000858306885, 0.4189999997615814, 0.4189999997615814, 0.4189999997615814, 0.41999998688697815, 0.42100000381469727, 0.41999998688697815, 0.42100000381469727, 0.42100000381469727, 0.42100000381469727, 0.42100000381469727, 0.42500001192092896, 0.42100000381469727, 0.4259999990463257, 0.4259999990463257, 0.4259999990463257, 0.4300000071525574, 0.4300000071525574, 0.42100000381469727, 0.4339999854564667, 0.4230000078678131, 0.4230000078678131, 0.44200000166893005, 0.44200000166893005, 0.42800000309944153, 0.42800000309944153, 0.42800000309944153, 0.42800000309944153, 0.44200000166893005, 0.4480000138282776, 0.4490000009536743, 0.4490000009536743, 0.4490000009536743, 0.4440000057220459, 0.4440000057220459, 0.4440000057220459, 0.4519999921321869, 0.45399999618530273, 0.45399999618530273, 0.4560000002384186, 0.4560000002384186, 0.4569999873638153, 0.4659999907016754, 0.46700000762939453, 0.46700000762939453, 0.46700000762939453, 0.4690000116825104, 0.47099998593330383, 0.4690000116825104, 0.4779999852180481, 0.4699999988079071, 0.4790000021457672, 0.4790000021457672, 0.4869999885559082, 0.4860000014305115, 0.48899999260902405, 0.49300000071525574, 0.49399998784065247, 0.4950000047683716, 0.5, 0.5, 0.5, 0.5099999904632568, 0.5120000243186951, 0.5099999904632568, 0.5120000243186951, 0.5139999985694885, 0.5170000195503235, 0.5170000195503235, 0.5170000195503235, 0.5320000052452087, 0.5350000262260437, 0.5389999747276306, 0.5460000038146973, 0.546999990940094, 0.546999990940094, 0.5490000247955322, 0.5509999990463257, 0.5550000071525574, 0.5609999895095825, 0.5630000233650208, 0.5669999718666077, 0.574999988079071, 0.574999988079071, 0.574999988079071, 0.578000009059906, 0.5820000171661377, 0.5839999914169312, 0.593999981880188, 0.593999981880188, 0.6010000109672546, 0.6029999852180481, 0.6079999804496765, 0.6179999709129333, 0.6209999918937683, 0.6240000128746033, 0.6359999775886536, 0.6449999809265137, 0.6480000019073486, 0.6480000019073486, 0.6499999761581421, 0.6520000100135803, 0.6579999923706055, 0.6759999990463257, 0.6850000023841858, 0.6880000233650208, 0.6959999799728394, 0.7009999752044678, 0.7020000219345093, 0.7139999866485596, 0.722000002861023, 0.7350000143051147, 0.7350000143051147, 0.7390000224113464, 0.7490000128746033, 0.7519999742507935, 0.7580000162124634, 0.7599999904632568, 0.7919999957084656, 0.7960000038146973, 0.7960000038146973, 0.7990000247955322, 0.8119999766349792, 0.8289999961853027, 0.8320000171661377, 0.8360000252723694, 0.8489999771118164, 0.8550000190734863, 0.8730000257492065, 0.878000020980835, 0.8970000147819519, 0.9129999876022339, 0.9340000152587891, 0.9440000057220459, 0.9449999928474426, 0.9559999704360962, 0.9829999804496765, 0.9900000095367432, 1.0099999904632568, 1.031999945640564, 1.0369999408721924, 1.0499999523162842, 1.0729999542236328, 1.0829999446868896, 1.1150000095367432, 1.1230000257492065, 1.1540000438690186, 1.1629999876022339, 1.1829999685287476, 1.2139999866485596, 1.2319999933242798, 1.2730000019073486, 1.2899999618530273, 1.3040000200271606, 1.340999960899353, 1.3830000162124634, 1.4110000133514404, 1.4559999704360962, 1.4889999628067017, 1.5219999551773071, 1.562999963760376, 1.6100000143051147, 1.6640000343322754, 1.7029999494552612, 1.7589999437332153, 1.8140000104904175, 1.8519999980926514, 1.9170000553131104, 1.9600000381469727, 2.046999931335449, 2.1059999465942383, 2.177000045776367, 2.26200008392334, 2.3320000171661377, 2.430999994277954, 2.51200008392334, 2.617000102996826, 2.73799991607666, 2.885999917984009, 3.0209999084472656, 3.1640000343322754, 3.312999963760376, 3.507999897003174, 3.7019999027252197, 3.9140000343322754, 4.076000213623047, 4.117000102996826, 4.171000003814697, 4.171000003814697, 4.170000076293945, 4.170000076293945, 4.170000076293945, 4.168000221252441, 4.164000034332275, 4.163000106811523, 4.163000106811523, 4.160999774932861, 4.160999774932861, 4.163000106811523, 4.159999847412109, 4.165999889373779, 4.165999889373779, 4.170000076293945, 4.170000076293945, 4.170000076293945, 4.169000148773193, 4.1620001792907715, 4.1620001792907715, 4.1620001792907715, 4.1620001792907715, 4.1620001792907715, 4.168000221252441, 4.171000003814697, 4.175000190734863, 4.176000118255615, 4.176000118255615, 4.176000118255615, 4.176000118255615, 4.182000160217285, 4.184000015258789, 4.185999870300293, 4.199999809265137, 4.199999809265137, 4.202000141143799, 4.214000225067139, 4.214000225067139, 4.2170000076293945, 4.2170000076293945, 4.2230000495910645, 4.22599983215332, 4.236000061035156, 4.241000175476074, 4.249000072479248, 4.25, 4.250999927520752, 4.26200008392334, 4.27400016784668, 4.27400016784668, 4.2769999504089355, 4.284999847412109, 4.285999774932861, 4.285999774932861, 4.285999774932861, 2.433000087738037, 2.384999990463257, 2.384999990463257, 2.361999988555908, 2.311000108718872, 2.311000108718872, 2.316999912261963, 2.309999942779541, 2.316999912261963, 2.319999933242798, 2.322999954223633, 2.3259999752044678, 2.3369998931884766, 2.3450000286102295, 2.3499999046325684, 2.3589999675750732, 2.3589999675750732, 2.3589999675750732, 2.359999895095825, 4.514999866485596, 4.514999866485596, 4.5229997634887695, 4.543000221252441, 4.545000076293945, 4.552000045776367, 4.566999912261963, 4.578999996185303, 4.586999893188477, 4.607999801635742, 4.632999897003174, 4.632999897003174, 4.651000022888184, 4.665999889373779, 4.682000160217285, 4.695000171661377, 4.705999851226807, 4.732999801635742, 4.739999771118164, 4.767000198364258, 4.769999980926514, 4.7820000648498535, 4.796000003814697, 4.809999942779541, 4.833000183105469, 4.855000019073486, 4.861999988555908, 4.89300012588501, 4.90500020980835, 4.919000148773193, 4.935999870300293, 4.949999809265137, 4.964000225067139, 4.964000225067139, 4.964000225067139, 3.513000011444092, 3.506999969482422, 3.4679999351501465, 3.4570000171661377, 3.4189999103546143, 3.390000104904175, 3.365000009536743, 3.3380000591278076, 3.312999963760376, 3.2809998989105225, 3.2660000324249268, 3.234999895095825, 3.2119998931884766, 3.187000036239624, 3.1659998893737793, 3.140000104904175, 3.140000104904175, 3.140000104904175, 3.1410000324249268, 3.1700000762939453, 3.177999973297119, 3.177999973297119, 3.187000036239624, 3.2300000190734863, 3.2309999465942383, 3.24399995803833, 3.265000104904175, 3.2799999713897705, 3.3010001182556152, 3.315999984741211, 3.3440001010894775, 3.38100004196167, 3.3940000534057617, 3.4189999103546143, 3.433000087738037, 5.599999904632568, 5.599999904632568, 5.599999904632568, 4.861000061035156, 4.854000091552734, 4.836999893188477, 4.803999900817871, 4.802999973297119, 4.76200008392334, 4.741000175476074, 1.9930000305175781, 1.968999981880188, 1.965000033378601, 1.965000033378601, 1.9639999866485596, 1.9429999589920044, 1.9390000104904175, 1.9290000200271606, 1.9270000457763672, 1.9129999876022339, 1.9119999408721924, 1.906999945640564, 1.8980000019073486, 1.8949999809265137, 1.8799999952316284, 1.878000020980835, 1.8630000352859497, 1.8580000400543213, 1.8569999933242798, 1.8480000495910645, 1.8480000495910645, 1.8480000495910645, 1.8480000495910645, 1.8569999933242798, 1.8849999904632568, 4.177999973297119, 4.177999973297119, 4.177999973297119, 4.165999889373779, 4.150000095367432, 4.144999980926514, 4.124000072479248, 4.113999843597412, 4.098999977111816, 4.090000152587891, 4.066999912261963, 4.053999900817871, 4.051000118255615, 4.045000076293945, 4.039999961853027, 4.008999824523926, 3.994999885559082, 3.986999988555908, 3.9700000286102295, 3.9670000076293945, 3.9660000801086426, 3.9560000896453857, 3.946000099182129, 3.9189999103546143, 3.9159998893737793, 3.9089999198913574, 3.8940000534057617, 3.88700008392334, 3.88100004196167, 3.880000114440918, 3.86899995803833, 3.8369998931884766, 3.8320000171661377, 3.821000099182129, 3.819000005722046, 3.819000005722046, 3.816999912261963, 3.806999921798706, 3.799999952316284, 3.7980000972747803, 3.7909998893737793, 3.7829999923706055, 3.7769999504089355, 3.7750000953674316, 3.763000011444092, 3.760999917984009, 3.756999969482422, 3.750999927520752, 3.749000072479248, 3.749000072479248, 3.736999988555908, 3.736999988555908, 3.7219998836517334, 3.7200000286102295, 3.7160000801086426, 3.7139999866485596, 3.7139999866485596, 3.7070000171661377, 3.697000026702881, 3.690999984741211, 3.693000078201294, 3.693000078201294, 3.697000026702881, 3.693000078201294, 3.693000078201294, 3.678999900817871, 3.6760001182556152, 3.6649999618530273, 3.6649999618530273, 3.6649999618530273, 3.6649999618530273, 3.6640000343322754, 3.6640000343322754, 3.6579999923706055, 3.6559998989105225, 3.6579999923706055, 3.6559998989105225, 3.6500000953674316, 3.640000104904175, 3.6500000953674316, 3.640000104904175, 3.640000104904175, 3.632999897003174, 3.632999897003174, 3.632999897003174, 3.632999897003174, 3.632999897003174, 3.628000020980835, 3.628000020980835, 3.628000020980835, 3.63100004196167};
    
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