# Teşhis araçları

Bunlar **otomatik test değil**, elle çalıştırılan teşhis araçlarıdır. `colcon test`
bunları çalıştırmaz; hepsi **çalışan bir simülasyon** ister ve bir kısmı robotu
gerçekten sürer.

```bash
# Önce simülasyonu başlat
ros2 launch unitree_go2_sim unitree_go2_launch.py rviz:=false
# Kontrolcüler aktifleşene kadar bekle (~35 sn), sonra:
python3 unitree_go2_sim/tools/<arac>.py
```

## Dünyaya bağımlılık — önemli

`odom_quality`, `check_scan`, `score_map`, `align_score`, `explore`, `tour` ve
`gait_stability` araçları `simple_room.sdf` parkurunun geometrisini bilir. Bunu
`unitree_go2_description/tools/gen_simple_room.py` içindeki tanımdan **canlı olarak**
okurlar; yani parkuru o üreteçten değiştirirsen araçlar kendiliğinden uyum sağlar.

Ama **başka bir dünyaya geçersen** (kendi yazdığın bir SDF, indirdiğin hazır bir
ortam) bu araçlar sessizce anlamsız sonuç üretir — hâlâ simple_room'un engellerine
göre hesap yaparlar. Öyle bir durumda yalnızca `scan_probe` ve `crop_map` geçerli
kalır.

---

## Araçlar

### `odom_quality.py` — odometri, gerçeğe karşı
`/odom` (EKF çıktısı) ile `/odom/ground_truth` (Gazebo'nun gerçeği) arasındaki farkı
ölçer. Düz yürüyüş ve yerinde dönüş ayrı raporlanır, çünkü bacaklı robot ikisinde
çok farklı davranır.

İki tuzağa karşı korumalı: engele 1 m kala durur (robot bir yere dayanıp odometri
saymaya devam ederse hata uydurulmuş olur) ve dönüşü adım adım biriktirir
(20 sn'lik dönüş 344° eder ve ±180°'de sararak baştan-sona karşılaştırmayı bozar).

Ölçülen tipik değerler: **dönüşte %0,3–0,5**, **düz yürüyüşte %2–18**. Öteleme hatası
mesafeyle büyüyor — 1,1 m'de %1,5, 1,9 m'de %18 görüldü — ve tek koşuda ayak kaymasına
göre epey değişiyor, bu yüzden tek bir sayıya bakma, birkaç kez çalıştır.

Dönüş hatası %5'i aşıyorsa gerçek bir sorun var: `/odom/raw` (CHAMP bacak odometrisi),
`/imu/data` ve `/odom` üçünü aynı anda karşılaştır, hata hangisinde başlıyor bul.
Öteleme hatası SLAM tarafından düzeltilebiliyor (harita %96 doğrulukla çıktı), ama
kalıcı olarak azaltmak istersen `gait.yaml` içindeki `odom_scaler` bunun katsayısı.

### `amcl_quality.py` — konumlandırma, gerçeğe karşı
AMCL'in tahminini Gazebo'nun gerçeğiyle karşılaştırır. Robotu çarpmadan gezdirir,
konum ve yön hatasının yanı sıra **parçacık bulutunun yayılımını** da raporlar.

Yayılım en önemli sinyal: bulut zamanla **daralmalı**. Açılıyorsa filtre güven
kazanamıyor demektir ve genelde `alpha` değerleri fazla yüksektir. Daralıyor ama
konum hatası büyükse filtre emin ama yanlıştır — bu ters yönde bir sorundur.

```bash
python3 amcl_quality.py 120      # saniye
```

Bu robotta ölçülen: konum medyanı **0,11–0,12 m**, yön medyanı **4°**, yayılım
0,61 → 0,32. `alpha`'lar ölçülen odometriye göre ayarlandı — dönüş terimleri düşük
(heading doğru), öteleme terimi yüksek (zayıf eksen).

### `kidnap_recovery.py` — kaçırılan robot deneyi
AMCL'i üç aşamada sınar: doğru tohum, yanlış tohum (kaçırılmış), sonra
`/reinitialize_global_localization`. Her aşamada konum hatasını ölçer.

Bu robotta ölçülen: doğru tohumla **0,11 m**, kaçırıldıktan sonra **2,35 m**
(kendiliğinden toparlayamıyor), küresel konumlandırmadan sonra tekrar **0,11 m**.

```bash
python3 kidnap_recovery.py
```

### `nav_test.py` — Nav2 hedefe gidiyor mu
İki hedef gönderir ve her birini gerçekle ölçer: süre, kat edilen yol, planın
uzunluğu, kaç kez yeniden planlandı, hedef hatası, **en yakın engele mesafe** ve
robot ayakta kaldı mı.

Plan uzunluğu ile kat edilen yolu ayrı raporlaması kasıtlı: kısa bir planı takip
edemiyorsa sorun kontrolcüde, uzun bir planı sadakatle izliyorsa sorun planlayıcı
veya costmap'te.

Hedefi göndermeden önce boş alanda mı diye bakar — engelin içindeki bir hedef,
navigasyon hatası gibi görünen ama olmayan bir başarısızlık üretir.

```bash
python3 nav_test.py
```

Bu robotta ölçülen: iki hedef de SUCCEEDED, en yakın engel **0,70–0,75 m**,
hedef hatası 0,20–0,56 m. Yol hâlâ kuş uçuşunun ~3 katı.

### `costmap_ghosts.py` — hayalet engel sayımı
Costmap'te dolu işaretlenmiş ama gerçekte boş olan hücreleri sayar. RViz'de boş
alanda beliren ve kaybolan camgöbeği lekelerin ölçülebilir hali.

Gövde eğimini de örnekler, çünkü en olası sebep lazer diliminin eğilip uzaktaki
zemin noktalarını banda sokmasıdır.

```bash
# baska bir terminalde robotu gezdirirken calistir
python3 costmap_ghosts.py 120
```

Bu robotta ölçülen: dururken **0**, yürürken başlangıçta ort **61** (güncellemelerin
%98'inde), `obstacle_max_range` 2,5 m'ye çekildikten sonra ort **1,1** (%10).

Not: eşik `100` olmalı, `99` değil. Nav2 yayınında 99 = şişirme kabuğu, 100 = gerçek
engel. 99'u saymak sıradan kabuğu binlerce hayalet gibi gösterir.

### `check_scan.py` — tarama, bilinen dünyaya karşı
Robotun **gerçek** pozundan bilinen engel geometrisini ışın-izler ve `/scan` ile
karşılaştırır. "Sensör verisi yanlış" ile "SLAM iyi veriyi yanlış işliyor" ayrımını
kesinleştirir — haritalama bozukken bakılacak ilk yer burasıdır.

Sağlıklı: ışınların %90'ından fazlası 10 cm içinde.

### `score_map.py` — harita, gerçeğe karşı
Kaydedilmiş bir haritayı üç ayrı açıdan ölçer. Gözle bakmaktan çok daha güvenilir;
ASCII önizleme kolayca yanıltır.

```bash
python3 score_map.py <harita>.pgm <harita>.yaml
```

**1. Doğruluk** — her dolu hücrenin gerçek yüzeye uzaklığı.
Sağlıklı: hücrelerin %85'inden fazlası 2 hücre (10 cm) içinde.

**2. Yüzey kapsaması** — gerçek yüzeylerin yüzde kaçının haritada karşılığı var.
Doğruluk tek başına eksik kalıyordu: "çizdiğim şey gerçekten duvar mı" diye sorar,
"duvarın tamamını çizdim mi" diye sormaz. İçinden ışın geçirilmiş, delik bir harita
yalnızca birinci ölçüde pekâlâ yüksek puan alır.

**3. Oda dışı boş hücre** — duvarın arkasında boş işaretlenmiş alan.
Oda kapalı, dışarısı erişilemez; oradaki her boş hücre bir duvarın delinmiş olduğu
anlamına gelir. En olası sebep `range_min`: robot duvara 0,5 m'den yakın geçtiğinde
o ışınlar atılır, SLAM ise ışının gittiği yeri boş sayar. Yani duvar silinip arkası
boşluk olarak işlenir. Çözüm haritalarken duvara yaklaşmamak — `explore.py` zaten
1 m'lik bir pay bırakır.

Ölçülen: elle sürülen ilk haritada 91 hücre (0,23 m²) sızıntı vardı.

### `align_score.py` — harita döndü mü?
`score_map` düşük puan verdiğinde çalıştır. Küçük dönüş ve kaymalar deneyerek en iyi
hizalamayı arar. En iyi hizalamada puan yükseliyorsa harita **kaymış**; yükselmiyorsa
harita **kendi içinde bozuk** — bu ikisi çok farklı sorunlardır.

```bash
python3 align_score.py <harita>.pgm <harita>.yaml
```

### `tour.py` — planlı kapsama turu (haritalama için)
Odayı biçerdöver deseninde baştan sona gezer. Dünya geometrisini 0,85 m şişirip
ızgaraya çevirir, durakları o ızgaraya dizer ve aralarını A\* ile planlar.

```bash
python3 tour.py          # tam tur (~13 dk)
python3 tour.py 600      # 600 sn sonra nerede kaldıysa bırak
```

Haritalama için `explore.py` yerine bunu kullan. `explore` refleksle çalışır ve bir
odayı **kapatamaz**: ölçülen iki koşuda toplam 12 dakikada gerçek yüzeyin ancak
%70'ini haritaladı, ikisi de başladığı noktanın bir metre yakınında bitti.

Şişirme payı yalnızca çarpmayı önlemek için değil. `range_min` 0,5 m olduğundan
duvara daha yakın geçilirse o ışınlar atılır ve SLAM ışının geçtiği yeri boş sayar
— duvar silinir, arkası zemin olur. Payı plana gömmek bunu tesadüfe değil tasarıma
bağlar.

**Hız ayarı ölçümle belirlendi**, tahminle değil:

| Komut | Gerçekleşen | Gövde eğimi |
|---|---|---|
| 0,15 m/s | 0,021 m/s | — |
| 0,25 m/s | 0,085 m/s | — |
| **0,35 m/s** | **0,141 m/s** | **12°** |
| 0,50 m/s | 0,149 m/s | 13° — *devrildi* |
| 0,70 m/s | 0,203 m/s | 15° |

`0.15` komutunda robot neredeyse yerinde sayıyor — istenenin %14'ü. `explore.py`
bu hızı kullandığı için 300 saniyede ancak 6 metre yol yapıyordu; "kapsama sorunu"
sanılan şey aslında buydu. `0.50` ise devirdi. `0.35` yer hızının %95'ini veriyor
ve gait çok daha sakin kalıyor. Ayrıca `RAMP` var: dönüş bitip tam gaza tek adımda
geçmek devrilmenin asıl sebebiydi.

Ölçülen: 36 durak, 779 sn, çarpma yok, devrilme yok, en yakın geçiş 0,49 m.

### `explore.py` — çarpmadan gezinti
Haritalama için robotu parkurda gezdirir. Önündeki boşluğu dünya tanımından okur ve
engele yaklaşınca daha açık tarafa döner. Sonunda en küçük clearance'ı ve çarpma olup
olmadığını raporlar.

```bash
python3 explore.py 250      # saniye
```

### `gait_stability.py` — devrilme testi
Sabit bir parkurda sürer ve robotun devrilip devrilmediğini, devrildiyse **nerede** ve
o noktada en yakın engele **ne kadar uzakta** olduğunu yazar. Bu ayrım önemli: açık
alanda devrilme yürüyüş ayarı sorunudur, engele 0,4 m'den yakınken devrilme çarpmadır.

### `scan_probe.py` — hızlı sağlık kontrolü
Bulut, tarama ve harita hakkında tek seferlik özet: nokta sayısı, geçerli ışın oranı,
mesafe aralığı, harita boyutu. Dünyadan bağımsız çalışır.

### `crop_map.py` — haritayı kırp
Kaydedilmiş haritanın kenarındaki tamamen keşfedilmemiş boşluğu atar. Piksel kırparken
`origin`'i de kaydırır — bu şart, yoksa harita dünyada sessizce kayar.

```bash
python3 crop_map.py girdi.pgm girdi.yaml cikti_taban_adi
```

### `stop_sim.sh` — her şeyi durdur
Simülasyona ait tüm süreçleri öldürür. Kendi komut satırını eşleştirmemek için
çalıştırılabilir dosyada tutulur — `pkill -f` ile aynı işi satır içinde yapmak, kabuğu
kendi kendine öldürüp temizliği yarıda bırakır.

Buna ihtiyaç duyulmasının sebebi gerçek: artık süreçler ölçümleri sessizce kirletir.
Bir noktada aynı anda beş `slam_toolbox` çalışıyor ve `/scan`'e iki farklı çözünürlükte
yayın yapılıyordu.

```bash
bash unitree_go2_sim/tools/stop_sim.sh
```

---

## Sorun ararken sıra

1. `stop_sim.sh` — artık süreç kalmadığından emin ol, sonra tek bir yığın başlat
2. `scan_probe.py` — veri akıyor mu, harita büyüyor mu
3. `check_scan.py` — tarama doğru mu? Değilse SLAM'e bakma, sensör zincirine bak
4. `odom_quality.py` — odometri doğru mu? Dönüş hatası tarama hatasından sonra gelir
5. `tour.py` ile gez, kaydet, `score_map.py` ile puanla
6. Puan düşükse `align_score.py` — kayma mı, bozulma mı
