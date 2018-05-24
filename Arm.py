import numpy as np
import scipy.optimize


class Arm3Link:

    def __init__(self, q=None, q0=None, L=None):
        "" " Kolun temel parametrelerini ayarlayýn.
        Tüm listeler sýrayla [omuz, dirsek, bilek].
        q: np.array
            kolun ilk eklem açýlarý
        q0: np.array
            varsayýlan (dinlenme durumu) eklem yapýlandýrmasý
        L: np.array
            kol segmenti uzunluklarý
        """
        #  baþlangýç ??eklem açýlarý
        self.q = [.3, .3, 0] if q is None else q
        # bazý varsayýlan kol pozisyonlarý
        self.q0 = np.array([np.pi/4, np.pi/4, np.pi/4]) if q0 is None else q0
        # kol segmenti uzunluklarý
        self.L = np.array([1, 1, 1]) if L is None else L

        self.max_angles = [np.pi, np.pi, np.pi/4]
        self.min_angles = [0, 0, -np.pi/4]

    def get_xy(self, q=None):
        "" "  karþýlýk gelen el xy koordinatlarýný döndürür
        belirli bir eklem açýsý deðerleri kümesi [omuz, dirsek, bilek],
        ve yukarýda tanýmlanan kol segmenti uzunluklarý, L
        q: np.array
            Mevcut baðlantý açýlarýnýn listesi
        döndürür: liste
            kolun [x, y] konumu
        """
        if q is None:
            q = self.q

        x = self.L[0]*np.cos(q[0]) + \
            self.L[1]*np.cos(q[0]+q[1]) + \
            self.L[2]*np.cos(np.sum(q))

        y = self.L[0]*np.sin(q[0]) + \
            self.L[1]*np.sin(q[0]+q[1]) + \
            self.L[2]*np.sin(np.sum(q))

        return [x, y]

    def inv_kin(self, xy):
         "" " Bu ters kinematiði bulmak için hýzlý bir yazýmdýr.
        3-kollu bir kol için, SciPy optimize paket minimizasyonu kullanarak
        Elin bir (x, y) pozisyonu verildiðinde, bir takým eklem açýlarý döndürün (q)
        kýsýtlamaya dayalý minimizasyon kullanarak, kýsýtlama el ile eþleþmektedir (x, y),
        Her bir eklemin mesafesini varsayýlan konumdan (q0) küçültün.
        xy: tuple
            kolun istenilen xy pozisyonuna
        döndürür: liste
            optimal [omuz, dirsek, bilek] açý konfigürasyonu
        """
        def distance_to_default(q, *args):
             "" " 
            q: np.array
                Mevcut baðlantý açýlarýnýn listesi
            döndürür: skaler
                Varsayýlan kol pozisyonuna öklid mesafesi bulma
            """
           
            weight = [1, 1, 1.3]
            return np.sqrt(np.sum([(qi - q0i)**2 * wi
                           for qi, q0i, wi in zip(q, self.q0, weight)]))

        def x_constraint(q, xy):
            """Için karþýlýk gelen el xy koordinatlarýný döndürür
            belirli bir eklem açýsý deðerleri kümesi [omuz, dirsek, bilek],
            ve yukarýda tanýmlanan kol segmenti uzunluklarý, L
            q : np.array
                Mevcut baðlantý açýlarýnýn listesi
            xy : np.array
               mevcut xy pozisyonu (kullanýlmaz)
            returns : np.array
               
            """
            x = (self.L[0]*np.cos(q[0]) + self.L[1]*np.cos(q[0]+q[1]) +
                 self.L[2]*np.cos(np.sum(q))) - xy[0]
            return x

        def y_constraint(q, xy):
             "" " karþýlýk gelen el xy koordinatlarýný döndürür
            belirli bir eklem açýsý deðerleri kümesi [omuz, dirsek, bilek],
            ve yukarýda tanýmlanan kol segmenti uzunluklarý, L
            q: np.array
                Mevcut baðlantý açýlarýnýn listesi
            xy: np.array
                mevcut xy pozisyonu (kullanýlmaz)
            döndürür: np.array
                mevcut ve istenen x pozisyonu arasýndaki fark
            """
            y = (self.L[0]*np.sin(q[0]) + self.L[1]*np.sin(q[0]+q[1]) +
                 self.L[2]*np.sin(np.sum(q))) - xy[1]
            return y

        def joint_limits_upper_constraint(q, xy):
            """
            q: np.array
                Mevcut baðlantý açýlarýnýn listesi
            xy: np.array
                mevcut xy pozisyonu (kullanýlmaz)
            döndürür: np.array
                mevcut ve istenen y pozisyonu arasýndaki fark
            """
            return self.max_angles - q

        def joint_limits_lower_constraint(q, xy):
             "" " Ýþlev minimizasyonu için kullanýlýr;
            Bu iþlevin baþarýyla iletilmesi için 0'dan büyük olmasý gerekir.
            q: np.array
                mevcut eklem açýlarý
            xy: np.array
                mevcut xy pozisyonu (kullanýlmaz)
            döndürür: np.array
                kýsýtlama eþleþtiðinde all> 0
            """
            return q - self.min_angles

        return scipy.optimize.fmin_slsqp(
            func=distance_to_default,
            x0=self.q,
            eqcons=[x_constraint,
                    y_constraint],
            Eklemler için min / max açýlarýný eklemek için 
            # ieqcons = [joint_limits_upper_constraint,
            #           joint_limits_lower_constraint],
            args=(xy,),
            iprint=0)  


def test():
    # ###########Test##################

    arm = Arm3Link()

    #  # istenen (x, y) el pozisyonlarý kümesi
    x = np.arange(-.75, .75, .05)
    y = np.arange(.25, .75, .05)

    #  Bilgilerin yazdýrýlmasý için # eþik noktasý, sorunlu noktalar bulmak için
    thresh = .025

    count = 0
    total_error = 0
    # belirtilen x ve y deðerleri aralýðýnda test et
    for xi in range(len(x)):
        for yi in range(len(y)):
            # inv_kin iþlevini farklý hedefler aralýðýnda test edin
            xy = [x[xi], y[yi]]
            # nv_kin iþlevini çalýþtýrýn, optimum eklem açýlarýný alýn
            q = arm.inv_kin(xy=xy)
            #  bu açýlarla verilen elin (x, y) konumunu bul
            actual_xy = arm.get_xy(q)
            # kök karesi hatasýný hesapla
            error = np.sqrt(np.sum((np.array(xy) - np.array(actual_xy))**2))
            # hata toplamý
            total_error += np.nan_to_num(error)

            # Hata yüksekse daha fazla bilgi yazdýrýn
            if np.sum(error) > thresh:
                print('-------------------------')
                print('Initial joint angles', arm.q)
                print('Final joint angles: ', q)
                print('Desired hand position: ', xy)
                print('Actual hand position: ', actual_xy)
                print('Error: ', error)
                print('-------------------------')

            count += 1

    print('\n---------Results---------')
    print('Total number of trials: ', count)
    print('Total error: ', total_error)
    print('-------------------------')

if __name__ == '__main__':
    test()