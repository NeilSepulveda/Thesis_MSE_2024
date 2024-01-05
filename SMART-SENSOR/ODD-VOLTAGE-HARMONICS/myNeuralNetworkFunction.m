function [y1] = myNeuralNetworkFunction(x1)
%MYNEURALNETWORKFUNCTION neural network simulation function.
%
% Auto-generated by MATLAB, 23-Nov-2023 22:04:46.
%
% [y1] = myNeuralNetworkFunction(x1) takes these arguments:
%   x = Qx5 matrix, input #1
% and returns:
%   y = Qx1 matrix, output #1
% where Q is the number of samples.

%#ok<*RPMT0>

% ===== NEURAL NETWORK CONSTANTS =====

% Input 1
x1_step1.xoffset = [214.52;214.1;1.22;5.6707311954966;49.7665];
x1_step1.gain = [0.0963855421686747;0.0965717044905842;0.506329113924051;1.54468796569046;4.38596491228074];
x1_step1.ymin = -1;

% Layer 1
b1 = [0.30105514426596297062;-2.6374865660952395707;1.0044587948589664972;-2.4505436501889574785;-4.6721640155326022992;0.44786890129275436223;2.5213847931299584459;0.52328782079828650708;-0.75350804906476986922;-1.424803496870644226;-0.01784122867752219524;1.0995675160496147793;1.4362712214276922573;-2.5342421961712622291;1.7984549436797547806;2.2887368105987402345;0.393772459014311349;-1.7753098217089131605;-1.4103208608109174538;-2.1122564417994431629;1.8107020662140598244;0.27705562151085671996;-0.092433667381413162079;-0.95431219362009978013;0.12239756673266366516;-0.54963433844812259466;0.73991743383282881918;6.0607721536406300444;-1.6606848131799505364;5.4426289628014785649;-0.94125033284696546509;-3.7931399401781913383;1.5362535354193682657;-0.045326545993393516087;-1.4244993132540959557;1.6828376815245200682;0.44780811135758358255;-2.5313087185991265393;-0.038567496537558462744;2.522947235532625232;3.3495254181383615766;0.4085378330717424622;-2.1572800334004220701;-0.82694397489838322723;1.3779549623068114705;3.3985271742190392708;1.6567691074952093366;-1.9402492572900060797;-0.20530309918395195368;-0.83526915448477712101;-2.9642623114218387137;-2.597260386524627318;-1.0341221693028010975;-2.4620636152365422689;2.5657903704391813449;-1.0776406518634698006;-0.32780031348295590732;-2.6838163621650639179;-0.46306763422060231994;-3.7275610720769827289];
IW1_1 = [-0.76706451396818586819 -0.7889247541563472188 4.3842527426570754301 -0.22980185769478078672 0.74636142568007601827;-0.2320482608475687436 -0.233947458790719931 4.1252848071970316113 -0.20022731396952372918 0.01809024340345345705;-0.10302853541216422384 -0.11148455643683224947 -0.49174542851587355052 0.99174720252889136951 -0.073834323678979366101;1.4461170551143465168 1.4583547628871766921 -4.3993736506854927981 -0.38878666429304320484 1.8074892659719337118;2.1413625804273377362 2.1475079143772579648 -2.8265386963467307169 0.31093493468789007439 0.57692365297166392502;1.4330800373058794062 1.4060148489032169294 1.5502633042538385144 -0.017045259657180414359 0.34779009057455956277;-3.2457377893504482103 -3.2097515376726510894 3.2540367959571367962 -0.31358781104440464071 0.22892725895398183256;-0.78002406104139399012 -0.83020909922848906692 3.6793871181518364999 -0.11983813418215048807 -1.4041547489432073537;0.25447049039900471179 0.19437847875706532053 -2.7573457828812095727 0.18666481094123654749 2.0055618823264476092;-0.005765306123001638812 0.011027868387733771871 2.3111163623205426276 0.060963294247569639606 0.085080307749870340284;-0.14823075028040266776 -0.13700817764085343464 3.9689187989337129103 0.0011870409579496001701 1.6050451654399175982;-1.3537224150944693335 -1.3204624698782390091 -1.1888690314818013416 -0.11589477625201773103 1.1188859497777696728;-1.2881903615392464868 -1.2982426913980045224 2.090212815817926284 0.41188833199103752625 0.83729388988324382481;-2.0933031386811200569 -2.0895529982035747629 0.95844370562879610542 -0.88550145996685203365 0.87416109637933192555;-1.2659996335817917945 -1.2335267899981099404 -0.44133043825329909549 0.12568936447183942651 -1.2201287962960132294;0.46035392676742659113 0.44365352689315806778 3.504065152848558018 -0.031709218286609740234 -0.28791106106137392961;0.0034313752379041134732 -0.018595313228688164359 1.5912552194829880925 -0.18015786204870962162 -1.710660434334182689;-1.3737825924368061159 -1.3069576261727571254 -5.2727205862138024628 0.27470751540324112039 1.5677470844023579932;-1.8815258606396911834 -1.8599189317577589087 1.7981764311450603167 0.010454330483107761654 2.4466416646015609615;1.7058283894065684461 1.7254319579342205504 0.46649954081274752626 0.012044690648083136025 0.84141727971668256458;3.3758041773269078512 3.3585056066199983071 -0.86724343766525557875 0.83431225476117731876 -2.2411876149152716309;-0.42660385478447215846 -0.4418426020267772758 -2.3465278339974706689 -0.11908479639944155593 -3.4386767368559367775;0.52109402100891089038 0.52226928764392988036 -4.5185229166064981854 0.22754493731652020161 -1.3564200679512481429;2.9586986749644359485 2.9518536958317116436 2.0872020837818854311 0.38042402842543432406 -1.6278943801997134422;0.32497560116697699062 0.3512893715609480827 0.15010818208187554079 -0.18643245613388439996 3.5589319249802726652;0.059231924077937044004 0.04775813166314157171 -0.54240679259141522106 0.17577663187405256906 2.2505143597743821893;0.15553423867709845196 0.15014765330312954128 -0.39590310390067423585 0.89849881496443395612 0.44867502313650048373;1.0731582571722821928 1.0833783154104632551 8.6500626255794070829 -0.14091337880825299811 1.1724191658903442281;0.4024765483681992384 0.39977289631634860267 -1.2618603102248266978 0.90967157520501173984 0.96559043365882291088;-0.00038796858867120947133 -0.0024430998098706590421 5.7540061667169624116 0.095546647988016250586 -1.143692399111952307;-1.2659566950969745047 -1.31325325293954287 -2.2276355877159059737 -0.067205080943527548132 0.51443188993385824404;5.2477054646255067993 5.2291683477951007575 -0.15963511432067875595 0.97199451865516695026 -0.91774929475961986025;-1.7101814899855418695 -1.7206586422593170838 -0.80202330514620756574 -0.11786839837048031232 1.4856387055208584158;-0.39691747436441926489 -0.37847358830718386136 1.3546033084947368685 0.026176220911860087481 -0.21053079103173727438;-0.41221322371214824676 -0.40212952754058312044 2.7576002779228780959 -0.40614570043428988022 -2.6924276190757012728;0.63811961925486915259 0.61945260540598479082 0.45977430039956956875 -0.16894152229012099342 2.8662330687971557808;-0.80754136600139414526 -0.77094634436624087659 3.1843682275512925628 -0.21477014160880758542 -0.29541896831203712237;-0.44283399661321831697 -0.53806793525695650882 -5.3539435299241171506 0.079495208979878426159 0.28589400607430870416;-2.6141638713000299177 -2.6239777864191848877 0.9305137719644849259 -0.76397329926576285075 2.0973141197070597208;0.56848361314348416773 0.58274123781942155365 0.61692780215800824273 0.070097365879519382692 4.3274493475220454641;-4.3691096778216582663 -4.3945919553035555083 4.7829139842546570804 -0.61842309178660326285 0.39215191788143738405;1.5535677320633964893 1.5550282071260428385 1.7796906403943857367 -0.21093028354576104366 1.0795987042434473935;2.8654525031125763768 2.8549218426502660506 -1.8791104380798482332 -0.11452079901736827994 -0.0059202158954324820075;-1.141331456490248808 -1.1674124904713381046 0.9818528983082531747 0.012052995155443805772 1.569036388254551273;1.4871042956150135605 1.4878804318509846194 3.6415951969010129652 -0.01255413715361473545 -0.59531867656491255225;0.81938698221275640954 0.73464520765891272447 7.9925594892420432913 -0.20035441761164426988 -0.58519880249239430992;1.0052653982243084929 1.0721723302781072107 4.380521014257596768 -0.25675705069384963108 -2.1009827476702622739;-1.5297000765753294704 -1.5276404672016568043 -1.681241099794930971 -0.42021132438059705105 4.8393893351812220516;0.021176629900605526552 0.00072281042837518661402 3.9746979229317123128 0.46138857830151930317 2.8120651273291423777;2.6551492275833377121 2.6510522210874767346 2.7938278623481891394 0.25863806770995056983 -1.6529965833737174652;1.0174788093930533517 1.0254096696325456506 -2.4581735081068107895 0.33960083748128183645 0.30724567083467518858;1.3521652553382152995 1.3560746211709662212 6.2961778813555007162 0.32318238309498048544 -1.9097844526001577226;-0.67396098814075111605 -0.66752169781339742993 2.2248951706039328258 -0.56183262418885071554 -2.6820650895928026536;0.89966937714189676445 0.8919054402259223302 2.7186586540553854263 0.16152748322125110025 -0.5913361905927464468;-4.0706873521605171362 -4.0986247099950423589 -0.64876161425697032037 -1.1265737443565575493 0.28484372990950929116;0.33204329756923339767 0.32454908786043423774 0.19992405987246622545 0.14419244975144188126 3.5059683425390573142;-0.059243988452275388179 -0.038639048915048879174 0.23416050586754777463 0.16187061518354303735 -3.1297568840297911308;4.8845270297043184016 4.8396391944552608066 0.49636169642698596549 1.4351301636321220911 -0.31453016756577628632;-0.80489448984075651072 -0.79784455049303604213 0.072414143621386914407 0.31114009082495791558 1.4961441253718312794;4.9642649915068357558 4.9726654549106150682 0.21831771979150241259 1.0064811484220945736 -0.7214988078784321246];

% Layer 2
b2 = 0.8482964585964586357;
LW2_1 = [2.1502861265293486959 -0.81916847601321229888 0.5586888656991851887 -0.36275931825996055036 -1.1023640161174357299 2.2640178336843961304 -3.0256808771127148283 1.6514517089852838172 2.0330999712653792244 3.0364181857373244711 2.3246669686471332916 1.7849253793208512686 0.39294919576037307118 -0.87675209727118441627 1.4182595296348889935 -3.7700482737195994964 3.4384393486378366234 1.3721547735208583507 -0.8467559702568038249 1.4739972400997161106 0.57349429420275277458 -0.32884642283372156335 2.4729326918450276196 0.57153930765688976123 -1.1337827412197254429 2.2306127888341098497 -1.3348699420240266456 0.88118514558792349067 -0.63359988911070863615 1.3744560997716626094 4.9284850667870871987 1.6219890057662540439 -1.3224381817345500778 -1.8867491642477256519 1.4097705579696946376 -1.9544347483219355333 -2.1201021748608721573 -4.1040664476098447366 -0.59303384052740848453 1.3142103076103766046 1.6040478349213982412 -1.0752036198423289282 -1.1723798814855634909 2.699396345609896386 2.5728496156757780788 -1.5240193651168580935 1.1958527000361116599 0.1328348496335615414 -0.65806320131897311221 -0.65187349055526155883 3.0422798569055844453 -0.40387321121752034481 -1.1058186751970209283 -1.3592381392864036904 -1.8619550632945258695 -0.75918485114233669186 -1.3347580959410692003 -1.2145648680450726875 -1.1068584330159987328 -2.0590871474742122871];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = 0.447447313078885;
y1_step1.xoffset = 1.9202;

% ===== SIMULATION ========

% Dimensions
Q = size(x1,1); % samples

% Input 1
x1 = x1';
xp1 = mapminmax_apply(x1,x1_step1);

% Layer 1
a1 = tansig_apply(repmat(b1,1,Q) + IW1_1*xp1);

% Layer 2
a2 = repmat(b2,1,Q) + LW2_1*a1;

% Output 1
y1 = mapminmax_reverse(a2,y1_step1);
y1 = y1';
end

% ===== MODULE FUNCTIONS ========

% Map Minimum and Maximum Input Processing Function
function y = mapminmax_apply(x,settings)
y = bsxfun(@minus,x,settings.xoffset);
y = bsxfun(@times,y,settings.gain);
y = bsxfun(@plus,y,settings.ymin);
end

% Sigmoid Symmetric Transfer Function
function a = tansig_apply(n,~)
a = 2 ./ (1 + exp(-2*n)) - 1;
end

% Map Minimum and Maximum Output Reverse-Processing Function
function x = mapminmax_reverse(y,settings)
x = bsxfun(@minus,y,settings.ymin);
x = bsxfun(@rdivide,x,settings.gain);
x = bsxfun(@plus,x,settings.xoffset);
end
