import numpy as np
import scipy.optimize


class Arm3Link:

    def __init__(self, q=None, q0=None, L=None):
        "" " Kolun temel parametrelerini ayarlay�n.
        T�m listeler s�rayla [omuz, dirsek, bilek].
        q: np.array
            kolun ilk eklem a��lar�
        q0: np.array
            varsay�lan (dinlenme durumu) eklem yap�land�rmas�
        L: np.array
            kol segmenti uzunluklar�
        """
        #  ba�lang�� ??eklem a��lar�
        self.q = [.3, .3, 0] if q is None else q
        # baz� varsay�lan kol pozisyonlar�
        self.q0 = np.array([np.pi/4, np.pi/4, np.pi/4]) if q0 is None else q0
        # kol segmenti uzunluklar�
        self.L = np.array([1, 1, 1]) if L is None else L

        self.max_angles = [np.pi, np.pi, np.pi/4]
        self.min_angles = [0, 0, -np.pi/4]

    def get_xy(self, q=None):
        "" "  kar��l�k gelen el xy koordinatlar�n� d�nd�r�r
        belirli bir eklem a��s� de�erleri k�mesi [omuz, dirsek, bilek],
        ve yukar�da tan�mlanan kol segmenti uzunluklar�, L
        q: np.array
            Mevcut ba�lant� a��lar�n�n listesi
        d�nd�r�r: liste
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
         "" " Bu ters kinemati�i bulmak i�in h�zl� bir yaz�md�r.
        3-kollu bir kol i�in, SciPy optimize paket minimizasyonu kullanarak
        Elin bir (x, y) pozisyonu verildi�inde, bir tak�m eklem a��lar� d�nd�r�n (q)
        k�s�tlamaya dayal� minimizasyon kullanarak, k�s�tlama el ile e�le�mektedir (x, y),
        Her bir eklemin mesafesini varsay�lan konumdan (q0) k���lt�n.
        xy: tuple
            kolun istenilen xy pozisyonuna
        d�nd�r�r: liste
            optimal [omuz, dirsek, bilek] a�� konfig�rasyonu
        """
        def distance_to_default(q, *args):
             "" " 
            q: np.array
                Mevcut ba�lant� a��lar�n�n listesi
            d�nd�r�r: skaler
                Varsay�lan kol pozisyonuna �klid mesafesi bulma
            """
           
            weight = [1, 1, 1.3]
            return np.sqrt(np.sum([(qi - q0i)**2 * wi
                           for qi, q0i, wi in zip(q, self.q0, weight)]))

        def x_constraint(q, xy):
            """I�in kar��l�k gelen el xy koordinatlar�n� d�nd�r�r
            belirli bir eklem a��s� de�erleri k�mesi [omuz, dirsek, bilek],
            ve yukar�da tan�mlanan kol segmenti uzunluklar�, L
            q : np.array
                Mevcut ba�lant� a��lar�n�n listesi
            xy : np.array
               mevcut xy pozisyonu (kullan�lmaz)
            returns : np.array
               
            """
            x = (self.L[0]*np.cos(q[0]) + self.L[1]*np.cos(q[0]+q[1]) +
                 self.L[2]*np.cos(np.sum(q))) - xy[0]
            return x

        def y_constraint(q, xy):
             "" " kar��l�k gelen el xy koordinatlar�n� d�nd�r�r
            belirli bir eklem a��s� de�erleri k�mesi [omuz, dirsek, bilek],
            ve yukar�da tan�mlanan kol segmenti uzunluklar�, L
            q: np.array
                Mevcut ba�lant� a��lar�n�n listesi
            xy: np.array
                mevcut xy pozisyonu (kullan�lmaz)
            d�nd�r�r: np.array
                mevcut ve istenen x pozisyonu aras�ndaki fark
            """
            y = (self.L[0]*np.sin(q[0]) + self.L[1]*np.sin(q[0]+q[1]) +
                 self.L[2]*np.sin(np.sum(q))) - xy[1]
            return y

        def joint_limits_upper_constraint(q, xy):
            """
            q: np.array
                Mevcut ba�lant� a��lar�n�n listesi
            xy: np.array
                mevcut xy pozisyonu (kullan�lmaz)
            d�nd�r�r: np.array
                mevcut ve istenen y pozisyonu aras�ndaki fark
            """
            return self.max_angles - q

        def joint_limits_lower_constraint(q, xy):
             "" " ��lev minimizasyonu i�in kullan�l�r;
            Bu i�levin ba�ar�yla iletilmesi i�in 0'dan b�y�k olmas� gerekir.
            q: np.array
                mevcut eklem a��lar�
            xy: np.array
                mevcut xy pozisyonu (kullan�lmaz)
            d�nd�r�r: np.array
                k�s�tlama e�le�ti�inde all> 0
            """
            return q - self.min_angles

        return scipy.optimize.fmin_slsqp(
            func=distance_to_default,
            x0=self.q,
            eqcons=[x_constraint,
                    y_constraint],
            Eklemler i�in min / max a��lar�n� eklemek i�in 
            # ieqcons = [joint_limits_upper_constraint,
            #           joint_limits_lower_constraint],
            args=(xy,),
            iprint=0)  


def test():
    # ###########Test##################

    arm = Arm3Link()

    #  # istenen (x, y) el pozisyonlar� k�mesi
    x = np.arange(-.75, .75, .05)
    y = np.arange(.25, .75, .05)

    #  Bilgilerin yazd�r�lmas� i�in # e�ik noktas�, sorunlu noktalar bulmak i�in
    thresh = .025

    count = 0
    total_error = 0
    # belirtilen x ve y de�erleri aral���nda test et
    for xi in range(len(x)):
        for yi in range(len(y)):
            # inv_kin i�levini farkl� hedefler aral���nda test edin
            xy = [x[xi], y[yi]]
            # nv_kin i�levini �al��t�r�n, optimum eklem a��lar�n� al�n
            q = arm.inv_kin(xy=xy)
            #  bu a��larla verilen elin (x, y) konumunu bul
            actual_xy = arm.get_xy(q)
            # k�k karesi hatas�n� hesapla
            error = np.sqrt(np.sum((np.array(xy) - np.array(actual_xy))**2))
            # hata toplam�
            total_error += np.nan_to_num(error)

            # Hata y�ksekse daha fazla bilgi yazd�r�n
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