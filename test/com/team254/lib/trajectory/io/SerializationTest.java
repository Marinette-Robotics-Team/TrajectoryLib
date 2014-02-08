/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package com.team254.lib.trajectory.io;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.Path.Waypoint;
import com.team254.lib.trajectory.PathGenerator;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.io.JavaSerializer;
import junit.framework.Assert;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 *
 * @author jarussell
 */
public class SerializationTest {

  String kGoldenOutput = "package com.team254.testing;\n"
          + "\n"
          + "import com.team254.lib.trajectory.Trajectory;\n"
          + "\n"
          + "public class Auto1 {\n"
          + "  public Trajectory.Segment[] Left = {\n"
          + "    new Trajectory.Segment(0.0025, 0.5, 50.0, 5000.0, 0.0, 0.01, 0.0024999999999999996, 12.5),\n"
          + "    new Trajectory.Segment(0.012499999999999975, 0.9999999999999974, 49.999999999999744, -2.5579538487363607E-11, 0.0, 0.01, 0.012499999999999975, 12.5),\n"
          + "    new Trajectory.Segment(0.03499999999999992, 2.2499999999999942, 124.99999999999969, 7499.9999999999945, 0.0, 0.01, 0.03499999999999992, 12.5),\n"
          + "    new Trajectory.Segment(0.0750000000000014, 4.000000000000147, 175.00000000001532, 5000.000000001563, 0.0, 0.01, 0.0750000000000014, 12.5),\n"
          + "    new Trajectory.Segment(0.13750000000000395, 6.250000000000255, 225.00000000001074, 4999.9999999995425, 0.0, 0.01, 0.13750000000000395, 12.5),\n"
          + "    new Trajectory.Segment(0.22750000000000756, 9.00000000000036, 275.0000000000106, 4999.999999999983, 0.0, 0.01, 0.22750000000000756, 12.5),\n"
          + "    new Trajectory.Segment(0.34999999999999953, 12.249999999999197, 324.99999999988364, 4999.999999987307, 0.0, 0.01, 0.34999999999999953, 12.5),\n"
          + "    new Trajectory.Segment(0.5099999999999506, 15.999999999995106, 374.9999999995909, 4999.999999970725, 0.0, 0.01, 0.5099999999999506, 12.5),\n"
          + "    new Trajectory.Segment(0.7124999999999493, 20.24999999999987, 425.00000000047623, 5000.000000088534, 0.0, 0.01, 0.7124999999999493, 12.5),\n"
          + "    new Trajectory.Segment(0.9625000000000462, 25.000000000009692, 475.0000000009823, 5000.000000050608, 0.0, 0.01, 0.9625000000000462, 12.5),\n"
          + "    new Trajectory.Segment(1.2625000000001452, 30.000000000009894, 500.00000000002024, 2499.9999999037923, 0.0, 0.01, 1.2625000000001452, 12.5),\n"
          + "    new Trajectory.Segment(1.612499999999795, 34.99999999996499, 499.9999999955097, -4.510525286605116E-7, 0.0, 0.01, 1.612499999999795, 12.5),\n"
          + "    new Trajectory.Segment(2.0124999999993953, 39.999999999960025, 499.9999999995033, 3.993591235484928E-7, 0.0, 0.01, 2.0124999999993953, 12.5),\n"
          + "    new Trajectory.Segment(2.462499999998945, 44.99999999995499, 499.9999999994962, -7.105427357601002E-10, 0.0, 0.01, 2.462499999998945, 12.5),\n"
          + "    new Trajectory.Segment(2.962499999998445, 49.99999999994999, 499.99999999950046, 4.263256414560601E-10, 0.0, 0.01, 2.962499999998445, 12.5),\n"
          + "    new Trajectory.Segment(3.512499999997895, 54.999999999945004, 499.9999999995012, 7.389644451905042E-11, 0.0, 0.01, 3.512499999997895, 12.5),\n"
          + "    new Trajectory.Segment(4.112499999997295, 59.99999999994001, 499.99999999950046, -7.389644451905042E-11, 0.0, 0.01, 4.112499999997295, 12.5),\n"
          + "    new Trajectory.Segment(4.762499999996645, 64.99999999993493, 499.99999999949193, -8.526512829121202E-10, 0.0, 0.01, 4.762499999996645, 12.5),\n"
          + "    new Trajectory.Segment(5.462499999998513, 70.0000000001868, 500.0000000251873, 2.5695385375001933E-6, 0.0, 0.01, 5.462499999998513, 12.5),\n"
          + "    new Trajectory.Segment(6.212500000001927, 75.00000000034142, 500.0000000154614, -9.725908967084251E-7, 0.0, 0.01, 6.212500000001927, 12.5),\n"
          + "    new Trajectory.Segment(7.010000000005556, 79.75000000036295, 475.00000000215294, -2500.0000013308463, 0.0, 0.01, 7.010000000005556, 12.5),\n"
          + "    new Trajectory.Segment(7.850000000009379, 84.00000000038226, 425.00000000193126, -5000.000000022169, 0.0, 0.01, 7.850000000009379, 12.5),\n"
          + "    new Trajectory.Segment(8.727500000013373, 87.75000000039945, 375.0000000017195, -5000.000000021174, 0.0, 0.01, 8.727500000013373, 12.5),\n"
          + "    new Trajectory.Segment(9.637500000017514, 91.00000000041408, 325.0000000014623, -5000.000000025721, 0.0, 0.01, 9.637500000017514, 12.5),\n"
          + "    new Trajectory.Segment(10.562007483009483, 92.45074829919687, 145.07482987827984, -17992.517012318243, 0.0010393981918701556, 0.01, 10.562007462811252, 12.500193253281836),\n"
          + "    new Trajectory.Segment(11.44382115069247, 88.18136676829873, -426.9381530898144, -57201.29829680943, 0.00729418889601674, 0.01, 11.443815233861772, 12.503418067607704),\n"
          + "    new Trajectory.Segment(12.26778148870941, 82.39603380169409, -578.5332966604642, -15159.514357064978, 0.01957694832585455, 0.01, 12.267707822745939, 12.513984079472941),\n"
          + "    new Trajectory.Segment(13.017470229790186, 74.96887410807756, -742.715969361653, -16418.267270118882, 0.038800930468247063, 0.01, 13.017094804407304, 12.535252841067058),\n"
          + "    new Trajectory.Segment(13.66749631720465, 65.00260874144634, -996.6265366631219, -25391.056730146884, 0.0665971859395631, 0.01, 13.66625980064263, 12.568700142634517),\n"
          + "    new Trajectory.Segment(14.178726571869975, 51.1230254665325, -1387.9583274913834, -39133.17908282615, 0.10569621676447727, 0.01, 14.175687382730612, 12.611594336800692),\n"
          + "    new Trajectory.Segment(14.489667587061087, 31.09410151911126, -2002.8923947421245, -61493.4067250741, 0.16081804560862456, 0.01, 14.48408187593667, 12.65130770342023),\n"
          + "    new Trajectory.Segment(14.495357680892385, 0.5690093831298624, -3052.5092135981395, -104961.6818856015, 0.24041033401778067, 0.01, 14.489578257014552, 12.64983564266887),\n"
          + "    new Trajectory.Segment(14.98653655527088, 49.11788743784939, 4854.887805471953, 790739.7019070092, 0.35972601462462817, 0.01, 14.021608350062024, 12.500634929458219),\n"
          + "    new Trajectory.Segment(16.299611242616166, 131.30746873452853, 8218.958129667913, 336407.0324195961, 0.5449192655621545, 0.01, 12.843603421487, 11.920574999106302),\n"
          + "    new Trajectory.Segment(18.791396593658213, 249.17853510420485, 11787.106636967632, 356814.85072997183, 0.8249096793782729, 0.01, 10.919057375653477, 10.33781893668408),\n"
          + "    new Trajectory.Segment(22.093079419742466, 330.16828260842544, 8098.974750422059, -368813.18865455734, 1.1703566356640551, 0.01, 9.12951526229176, 7.563175084121271),\n"
          + "    new Trajectory.Segment(24.798677854714203, 270.5598434971738, -5960.843911125164, -1405981.8661547222, 1.4675977863757135, 0.01, 8.447630803171009, 4.944912974298971),\n"
          + "    new Trajectory.Segment(26.266316772976722, 146.76389182625203, -12379.595167092177, -641875.1255967013, 1.6651963916852992, 0.01, 8.433475163535304, 3.4773423244611905),\n"
          + "    new Trajectory.Segment(26.794697047600376, 52.838027462365346, -9392.586436388669, 298700.87307035085, 1.7874905925548126, 0.01, 8.510489037594974, 2.9546047442034777),\n"
          + "    new Trajectory.Segment(26.849417982374078, 5.4720934773701, -4736.5933984995245, 465599.3037889144, 1.8631182452076807, 0.01, 8.493811052474662, 3.006722164658008),\n"
          + "    new Trajectory.Segment(27.272490053227365, 42.30720708532865, 3683.5113607958547, 842010.475929538, 1.9090767440653513, 0.01, 8.36073052910655, 3.4083184189186246),\n"
          + "    new Trajectory.Segment(27.94500707864164, 67.25170254142749, 2494.4495456098844, -118906.18151859703, 1.93449501760245, 0.01, 8.128161760386659, 4.039342124723865),\n"
          + "    new Trajectory.Segment(28.801168008867382, 85.61609302257392, 1836.439048114643, -65801.04974952414, 1.9442369536106938, 0.01, 7.818640091983196, 4.8375951392710945),\n"
          + "    new Trajectory.Segment(29.805860019980713, 100.46920111133299, 1485.3108088759072, -35112.82392387356, 1.9407106561193777, 0.01, 7.452811435799887, 5.773317016492132),\n"
          + "    new Trajectory.Segment(30.943390645258436, 113.75306252777214, 1328.3861416439152, -15692.466723199208, 1.9247703774228002, 0.01, 7.049084730874478, 6.836792766650449),\n"
          + "    new Trajectory.Segment(32.211966382789164, 126.85757375307264, 1310.4511225300498, -1793.5019113865337, 1.8961580217701233, 0.01, 6.625424066627028, 8.032533559389844),\n"
          + "    new Trajectory.Segment(33.620721475681535, 140.87550928923687, 1401.793553616423, 9134.243108637314, 1.8537403585569943, 0.01, 6.202385483476217, 9.376270612877594),\n"
          + "    new Trajectory.Segment(35.186855304826516, 156.61338291449803, 1573.7873625261159, 17199.380890969293, 1.795737304550812, 0.01, 5.807625711607615, 10.891836478588012),\n"
          + "    new Trajectory.Segment(36.929833992214306, 174.29786873877893, 1768.4485824280898, 19466.121990197393, 1.7201850737436004, 0.01, 5.481821984808715, 12.60409423847465),\n"
          + "    new Trajectory.Segment(38.85841583479712, 192.85818425828168, 1856.0315519502751, 8758.296952218529, 1.6259746963152237, 0.01, 5.283357790624937, 14.522437244083166),\n"
          + "    new Trajectory.Segment(40.95545677302328, 209.70409382261604, 1684.5909564334363, -17144.05955168388, 1.5142643936412428, 0.01, 5.283383947563712, 16.619478182146196),\n"
          + "    new Trajectory.Segment(43.15786885512853, 220.24120821052463, 1053.7114387908587, -63087.95176425776, 1.390098418309217, 0.01, 5.543044580451969, 18.80652992710589),\n"
          + "    new Trajectory.Segment(45.34642185605646, 218.85530009279307, -138.59081177315602, -119230.22505640147, 1.263036580201089, 0.01, 6.071879315188131, 20.930229072749448),\n"
          + "    new Trajectory.Segment(47.379927862283644, 203.3506006227182, -1550.4699470074881, -141187.9135234332, 1.1444009174379026, 0.01, 6.8018038888541845, 22.828216637018627),\n"
          + "    new Trajectory.Segment(49.150593792787035, 177.06659305033872, -2628.4007572379464, -107793.08102304583, 1.0428270116244023, 0.01, 7.615391415701723, 24.400900067847456),\n"
          + "    new Trajectory.Segment(50.61308269753535, 146.2488904748313, -3081.770257550741, -45336.950031279455, 0.9619362954963078, 0.01, 8.403026006703977, 25.633176638901467),\n"
          + "    new Trajectory.Segment(51.774352113990325, 116.12694164549775, -3012.194882933356, 6957.537461738502, 0.901160756722822, 0.01, 9.096136404327392, 26.56491943297073),\n"
          + "    new Trajectory.Segment(52.6688270125547, 89.44748985643723, -2667.945178906052, 34424.970402730374, 0.857737464390332, 0.01, 9.666646282641496, 27.253834393548427),\n"
          + "    new Trajectory.Segment(53.33939540960289, 67.0568397048195, -2239.0650151617733, 42888.01637442789, 0.8282303727335031, 0.01, 10.112870996038534, 27.754379576597387),\n"
          + "    new Trajectory.Segment(53.82784753517784, 48.84521255749461, -1821.1627147324882, 41790.23004292851, 0.80929394735562, 0.01, 10.446625720737346, 28.111021221277174),\n"
          + "    new Trajectory.Segment(54.1750418636305, 34.719432845266006, -1412.5779712228607, 40858.47435096275, 0.7978515625113651, 0.01, 10.687670584186543, 28.360903744704586),\n"
          + "    new Trajectory.Segment(54.41768657543454, 24.26447118040409, -1045.4961664861914, 36708.18047366692, 0.791359231770026, 0.01, 10.857681358842854, 28.534030266841512),\n"
          + "    new Trajectory.Segment(54.58403887101899, 16.63522955844551, -762.9241621958581, 28257.20042903333, 0.7879562891106177, 0.01, 10.974818239191492, 28.652149010097638),\n"
          + "    new Trajectory.Segment(54.69528916889497, 11.125029787598168, -551.0199770847341, 21190.4185111124, 0.7863474787636331, 0.01, 11.053350140286883, 28.730948561415133),\n"
          + "    new Trajectory.Segment(54.76702619016371, 7.173702126873859, -395.1327660724308, 15588.721101230334, 0.7856856984117362, 0.01, 11.10404591754, 28.781704234883364),\n"
          + "    new Trajectory.Segment(54.810607406666875, 4.358121650316503, -281.55804765573566, 11357.471841669512, 0.7854623495480133, 0.01, 11.134857454102042, 28.812525844941597),\n"
          + "    new Trajectory.Segment(54.83441652207827, 2.3809115411396373, -197.72101091768653, 8383.703673804914, 0.7854067357864941, 0.01, 11.151692498827371, 28.829361974121706),\n"
          + "    new Trajectory.Segment(54.844957271986765, 1.0540749908495923, -132.68365502900448, 6503.735588868204, 0.7853985582733937, 0.01, 11.159145907647773, 28.836815436779386),\n"
          + "    new Trajectory.Segment(54.847725321858015, 0.27680498712526064, -77.72700037243317, 5495.665465657132, 0.7853981637712915, 0.01, 11.161103214216388, 28.83877274388002),\n"
          + "    new Trajectory.Segment(54.84781303272921, 0.008771087120203285, -26.803390000505733, 5092.361037192743, 0.7853981633974465, 0.01, 11.161165235168157, 28.838834764831873),\n"
          + "    new Trajectory.Segment(54.84781303272921, 0.0, -0.8771087120203285, 2592.6281288485407, 0.7853981633974465, 0.01, 11.161165235168157, 28.838834764831873),\n"
          + "  };\n"
          + "\n"
          + "  public Trajectory.Segment[] Right = {\n"
          + "    new Trajectory.Segment(0.0025, 0.5, 50.0, 5000.0, 0.0, 0.01, 0.0024999999999999996, -12.5),\n"
          + "    new Trajectory.Segment(0.012499999999999975, 0.9999999999999974, 49.999999999999744, -2.5579538487363607E-11, 0.0, 0.01, 0.012499999999999975, -12.5),\n"
          + "    new Trajectory.Segment(0.03499999999999992, 2.2499999999999942, 124.99999999999969, 7499.9999999999945, 0.0, 0.01, 0.03499999999999992, -12.5),\n"
          + "    new Trajectory.Segment(0.0750000000000014, 4.000000000000147, 175.00000000001532, 5000.000000001563, 0.0, 0.01, 0.0750000000000014, -12.5),\n"
          + "    new Trajectory.Segment(0.13750000000000395, 6.250000000000255, 225.00000000001074, 4999.9999999995425, 0.0, 0.01, 0.13750000000000395, -12.5),\n"
          + "    new Trajectory.Segment(0.22750000000000756, 9.00000000000036, 275.0000000000106, 4999.999999999983, 0.0, 0.01, 0.22750000000000756, -12.5),\n"
          + "    new Trajectory.Segment(0.34999999999999953, 12.249999999999197, 324.99999999988364, 4999.999999987307, 0.0, 0.01, 0.34999999999999953, -12.5),\n"
          + "    new Trajectory.Segment(0.5099999999999506, 15.999999999995106, 374.9999999995909, 4999.999999970725, 0.0, 0.01, 0.5099999999999506, -12.5),\n"
          + "    new Trajectory.Segment(0.7124999999999493, 20.24999999999987, 425.00000000047623, 5000.000000088534, 0.0, 0.01, 0.7124999999999493, -12.5),\n"
          + "    new Trajectory.Segment(0.9625000000000462, 25.000000000009692, 475.0000000009823, 5000.000000050608, 0.0, 0.01, 0.9625000000000462, -12.5),\n"
          + "    new Trajectory.Segment(1.2625000000001452, 30.000000000009894, 500.00000000002024, 2499.9999999037923, 0.0, 0.01, 1.2625000000001452, -12.5),\n"
          + "    new Trajectory.Segment(1.612499999999795, 34.99999999996499, 499.9999999955097, -4.510525286605116E-7, 0.0, 0.01, 1.612499999999795, -12.5),\n"
          + "    new Trajectory.Segment(2.0124999999993953, 39.999999999960025, 499.9999999995033, 3.993591235484928E-7, 0.0, 0.01, 2.0124999999993953, -12.5),\n"
          + "    new Trajectory.Segment(2.462499999998945, 44.99999999995499, 499.9999999994962, -7.105427357601002E-10, 0.0, 0.01, 2.462499999998945, -12.5),\n"
          + "    new Trajectory.Segment(2.962499999998445, 49.99999999994999, 499.99999999950046, 4.263256414560601E-10, 0.0, 0.01, 2.962499999998445, -12.5),\n"
          + "    new Trajectory.Segment(3.512499999997895, 54.999999999945004, 499.9999999995012, 7.389644451905042E-11, 0.0, 0.01, 3.512499999997895, -12.5),\n"
          + "    new Trajectory.Segment(4.112499999997295, 59.99999999994001, 499.99999999950046, -7.389644451905042E-11, 0.0, 0.01, 4.112499999997295, -12.5),\n"
          + "    new Trajectory.Segment(4.762499999996645, 64.99999999993493, 499.99999999949193, -8.526512829121202E-10, 0.0, 0.01, 4.762499999996645, -12.5),\n"
          + "    new Trajectory.Segment(5.462499999998513, 70.0000000001868, 500.0000000251873, 2.5695385375001933E-6, 0.0, 0.01, 5.462499999998513, -12.5),\n"
          + "    new Trajectory.Segment(6.212500000001927, 75.00000000034142, 500.0000000154614, -9.725908967084251E-7, 0.0, 0.01, 6.212500000001927, -12.5),\n"
          + "    new Trajectory.Segment(7.010000000005556, 79.75000000036295, 475.00000000215294, -2500.0000013308463, 0.0, 0.01, 7.010000000005556, -12.5),\n"
          + "    new Trajectory.Segment(7.850000000009379, 84.00000000038226, 425.00000000193126, -5000.000000022169, 0.0, 0.01, 7.850000000009379, -12.5),\n"
          + "    new Trajectory.Segment(8.727500000013373, 87.75000000039945, 375.0000000017195, -5000.000000021174, 0.0, 0.01, 8.727500000013373, -12.5),\n"
          + "    new Trajectory.Segment(9.637500000017514, 91.00000000041408, 325.0000000014623, -5000.000000025721, 0.0, 0.01, 9.637500000017514, -12.5),\n"
          + "    new Trajectory.Segment(10.587992435416874, 95.04924353993599, 404.92435395219104, 7992.435395072875, 0.0010393981918701556, 0.01, 10.587992412929204, -12.499793242361864),\n"
          + "    new Trajectory.Segment(11.626175598548553, 103.81831631316791, 876.9072773231926, 47198.29233710015, 0.00729418889601674, 0.01, 11.626168339230167, -12.495916870445399),\n"
          + "    new Trajectory.Segment(12.757202950190251, 113.10273516416977, 928.4418851001859, 5153.460777699331, 0.01957694832585455, 0.01, 12.75710026898871, -12.48122536220929),\n"
          + "    new Trajectory.Segment(13.987483747139812, 123.02807969495612, 992.5344530786347, 6409.256797844875, 0.038800930468247063, 0.01, 13.986874687458041, -12.44593061725673),\n"
          + "    new Trajectory.Segment(15.332393600083787, 134.49098529439735, 1146.2905599441228, 15375.610686548816, 0.0665971859395631, 0.01, 15.329959010142224, -12.37588053013508),\n"
          + "    new Trajectory.Segment(16.821036592500235, 148.86429924164466, 1437.3313947247311, 29104.083478060827, 0.10569621676447727, 0.01, 16.81317552688893, -12.248889493757076),\n"
          + "    new Trajectory.Segment(18.509845460634764, 168.8808868134529, 2001.658757180823, 56432.73624560918, 0.16081804560862456, 0.01, 18.487225629943996, -12.02610788421277),\n"
          + "    new Trajectory.Segment(20.50422191485674, 199.43764542219742, 3055.675860874453, 105401.710369363, 0.24041033401778067, 0.01, 20.442107742472608, -11.631173247506029),\n"
          + "    new Trajectory.Segment(22.99418757950416, 248.996566464742, 4955.892104254457, 190021.6243380004, 0.35972601462462817, 0.01, 22.822053300399396, -10.899197733962293),\n"
          + "    new Trajectory.Segment(26.304355984056105, 331.0168404551946, 8202.02739904526, 324613.52947908035, 0.5449192655621545, 0.01, 25.80232968633568, -9.458653555561453),\n"
          + "    new Trajectory.Segment(30.789510668638364, 448.5154684582259, 11749.862800303128, 354783.54012578673, 0.8249096793782729, 0.01, 29.281219664740906, -6.627763755024905),\n"
          + "    new Trajectory.Segment(36.08112499128693, 529.161432264856, 8064.596380663011, -368526.6419640117, 1.1703566356640551, 0.01, 32.15175729188, -2.182407090949678),\n"
          + "    new Trajectory.Segment(40.77924795384689, 469.8122962559964, -5934.9136008859605, -1399950.998154897, 1.4675977863757135, 0.01, 33.31462467400729, 2.3695264348081007),\n"
          + "    new Trajectory.Segment(44.243570092606404, 346.4322138759511, -12338.008238004528, -640309.4637118567, 1.6651963916852992, 0.01, 33.322165707237644, 5.833840366009644),\n"
          + "    new Trajectory.Segment(46.77066742823466, 252.70973356282568, -9372.248031312543, 296576.0206691984, 1.7874905925548126, 0.01, 32.925827160450304, 8.32966421798116),\n"
          + "    new Trajectory.Segment(48.715548123000275, 194.48806947656124, -5822.166408626444, 355008.16226861, 1.8631182452076807, 0.01, 32.43324443450606, 10.211132574072568),\n"
          + "    new Trajectory.Segment(50.28747815074131, 157.19300277410315, -3729.5066702458084, 209265.9738380635, 1.9090767440653513, 0.01, 31.943898780208595, 11.704954800019856),\n"
          + "    new Trajectory.Segment(51.5954341728629, 130.79560221215942, -2639.7400561943728, 108976.66140514356, 1.93449501760245, 0.01, 31.492848548052052, 12.932677343543299),\n"
          + "    new Trajectory.Segment(52.69514237227017, 109.97081994072684, -2082.478227143258, 55726.182905111455, 1.9442369536106938, 0.01, 31.09558123970968, 13.958122032234982),\n"
          + "    new Trajectory.Segment(53.61167704111499, 91.653466884482, -1831.735305624484, 25074.29215187742, 1.9407106561193777, 0.01, 30.761769486373367, 14.811705954605255),\n"
          + "    new Trajectory.Segment(54.350705154984375, 73.90281138693912, -1775.065549754288, 5666.975587019601, 1.9247703774228002, 0.01, 30.499149865962472, 15.502497882804034),\n"
          + "    new Trajectory.Segment(54.903996979169555, 55.32918241851785, -1857.3628968421267, -8229.734708783872, 1.8961580217701233, 0.01, 30.31380336779123, 16.023821696115704),\n"
          + "    new Trajectory.Segment(55.25239152619585, 34.839454702629226, -2048.9727715888625, -19160.987474673584, 1.8537403585569943, 0.01, 30.208327353303698, 16.355866227414566),\n"
          + "    new Trajectory.Segment(55.36865959301086, 11.626806681500852, -2321.2648021128375, -27229.203052397497, 1.795737304550812, 0.01, 30.177807551837354, 16.468057158774254),\n"
          + "    new Trajectory.Segment(55.514045939257414, 14.538634624655499, 291.18279431546466, 261244.75964283018, 1.7201850737436004, 0.01, 30.20337792799373, 16.32493711986235),\n"
          + "    new Trajectory.Segment(55.939856076806244, 42.58101375488311, 2804.237913022761, 251305.5118707296, 1.6259746963152237, 0.01, 30.245309290013292, 15.901196592984713),\n"
          + "    new Trajectory.Segment(56.634122652417815, 69.42665756115723, 2684.564380627412, -11967.353239534896, 1.5142643936412428, 0.01, 30.24344634218526, 15.206932516831547),\n"
          + "    new Trajectory.Segment(57.533867026380165, 89.97443739623515, 2054.7779835077918, -62978.63971196202, 1.390098418309217, 0.01, 30.136007253860527, 14.313625861322446),\n"
          + "    new Trajectory.Segment(58.519723668649824, 98.58566422696613, 861.1226830730984, -119365.53004346933, 1.263036580201089, 0.01, 29.897244020422857, 13.357118967742688),\n"
          + "    new Trajectory.Segment(59.45037025045708, 93.06465818072549, -552.1006046240643, -141322.32876971626, 1.1444009174379026, 0.01, 29.563366136143333, 12.488425340150913),\n"
          + "    new Trajectory.Segment(60.21796062954946, 76.75903790923827, -1630.5620271487214, -107846.14225246571, 1.0428270116244023, 0.01, 29.21118815885449, 11.806394796562191),\n"
          + "    new Trajectory.Segment(60.77718857990662, 55.92279503571598, -2083.6242873522297, -45306.22602035082, 0.9619362954963078, 0.01, 28.910539409219695, 11.334858764212909),\n"
          + "    new Trajectory.Segment(61.13507407202712, 35.788549212050135, -2013.4245823665842, 7019.970498564544, 0.901160756722822, 0.01, 28.697334396952034, 11.047411989636675),\n"
          + "    new Trajectory.Segment(61.32609644025649, 19.10223682293722, -1668.6312389112916, 34479.33434552926, 0.857737464390332, 0.01, 28.57575781832044, 10.900073329766993),\n"
          + "    new Trajectory.Segment(61.393178850499154, 6.708241024265868, -1239.3995798671351, 42923.16590441565, 0.8282303727335031, 0.01, 28.53126943957024, 10.849865422818553),\n"
          + "    new Trajectory.Segment(61.40822780636577, 1.5048955866615799, -520.3345437604288, 71906.50361067062, 0.80929394735562, 0.01, 28.541630012897215, 10.860780073202864),\n"
          + "    new Trajectory.Segment(61.46936409901453, 6.113629264875563, 460.8733678213983, 98120.7911581827, 0.7978515625113651, 0.01, 28.584110725648223, 10.904746370784087),\n"
          + "    new Trajectory.Segment(61.549700831313764, 8.033673229923531, 192.00439650479683, -26886.897131660146, 0.791359231770026, 0.01, 28.640413979957884, 10.962051991020331),\n"
          + "    new Trajectory.Segment(61.63097960210528, 8.12787707915136, 9.420384922782965, -18258.401158201385, 0.7879562891106177, 0.01, 28.697651579139603, 11.019758973451337),\n"
          + "    new Trajectory.Segment(61.70200964573111, 7.103004362583485, -102.48727165678754, -11190.76565795705, 0.7863474787636331, 0.01, 28.747793385198484, 11.070068678108008),\n"
          + "    new Trajectory.Segment(61.75720215851278, 5.519251278166964, -158.37530844165215, -5588.803678486461, 0.7856856984117362, 0.01, 28.78679766533154, 11.10911838487332),\n"
          + "    new Trajectory.Segment(61.79519965343498, 3.799749492219321, -171.95017859476425, -1357.4870153112101, 0.7854623495480133, 0.01, 28.81366160890824, 11.135991013249978),\n"
          + "    new Trajectory.Segment(61.817618424808586, 2.2418771373607442, -155.78723548585768, 1616.2943108906575, 0.7854067357864941, 0.01, 28.829513567702158, 11.151843984968178),\n"
          + "    new Trajectory.Segment(61.82795473688957, 1.0336312080987415, -120.82459292620027, 3496.264255965741, 0.7853985582733937, 0.01, 28.83682241779655, 11.159152887603543),\n"
          + "    new Trajectory.Segment(61.83071292420827, 0.27581873186972605, -75.78124762290155, 4504.334530329872, 0.7853981637712915, 0.01, 28.83877275048875, 11.16110322082501),\n"
          + "    new Trajectory.Segment(61.830800625733346, 0.008770152507719974, -26.70485793620061, 4907.638968670093, 0.7853981633974465, 0.01, 28.838834764831816, 11.16116523516815),\n"
          + "    new Trajectory.Segment(61.830800625733346, 0.0, -0.8770152507719974, 2582.7842685428614, 0.7853981633974465, 0.01, 28.838834764831816, 11.16116523516815),\n"
          + "  };\n"
          + "\n"
          + "}\n";

  public SerializationTest() {
  }

  @BeforeClass
  public static void setUpClass() {
  }

  @AfterClass
  public static void tearDownClass() {
  }

  @Before
  public void setUp() {
  }

  @After
  public void tearDown() {
  }

  @Test
  public void testJavaSerialization() {
    Path p = new Path(10);
    p.addWaypoint(new Waypoint(0, 0, 0));
    p.addWaypoint(new Waypoint(10, 0, 0));
    p.addWaypoint(new Waypoint(20, 20, Math.PI / 4));

    TrajectoryGenerator.Config config = new TrajectoryGenerator.Config();
    config.dt = .01;
    config.max_acc = 1000.0;
    config.max_jerk = 5000.0;
    config.max_vel = 100.0;

    Trajectory[] lr = PathGenerator.generateLeftAndRightFromPath(p, config,
            25.0);

    JavaSerializer js = new JavaSerializer();
    String[] names = {"Left", "Right"};
    String serialized = js.serialize("com.team254.testing.Auto1", names, lr);
    System.out.print(serialized);

    Assert.assertEquals(serialized, kGoldenOutput);
  }
}