import java.time.Duration;
import java.time.LocalTime;
import java.util.Deque;
import java.util.Map;

public class TestSimulator1000000b {
    public static void main(String[] args) {
        System.out.println("---------------------------------------------------------------");
        LocalTime start = LocalTime.now() ;
        Graph graph = new Graph("nodes_1000000.csv","edges_1000000.csv");
        LocalTime graphReady = LocalTime.now() ;
        Duration d = Duration.between(start,graphReady) ;
        System.out.println();
        System.out.println("Graphe prêt !");
        System.out.println("Temps de chargement : "+d.toMinutes()+"m"+d.toSecondsPart()+"s"+d.toNanosPart()+"ns");
        System.out.println();
        System.out.println("---------------------------------------------------------------");
        long[] idsDepartCrue = {3060525656l,12974725584l,1098529543} ;
        start = LocalTime.now() ;
        Localisation[] zoneInondee = graph.determinerZoneInondee(idsDepartCrue);
        LocalTime tempCalculZoneInondée = LocalTime.now() ;
        d = Duration.between(start, tempCalculZoneInondée);
        System.out.println();
        System.out.println("Temps de calcul pour la zone inondée : "+d.toMinutes()+"m"+d.toSecondsPart()+"s"+d.toNanosPart()+"ns");
        System.out.println();
        System.out.println("Zone inondée :");
        System.out.println("--------------");
        for (int i=0 ; i<zoneInondee.length ; i++) {
            System.out.println(i+1+") "+zoneInondee[i].getId()+" : altitude : "+zoneInondee[i].getAltitude());
        }
        System.out.println();
        System.out.println("---------------------------------------------------------------");
        long idDepartTrajet = 1132625454 ;
        long idDestination = 1670435997 ;
        start = LocalTime.now() ;
        Deque<Localisation> chemin = graph.trouverCheminLePlusCourtPourContournerLaZoneInondee(idDepartTrajet,idDestination,zoneInondee) ;
        LocalTime tempsCalculCheminPlusCourtPourEviterZoneInondee = LocalTime.now() ;
        d = Duration.between(start,tempsCalculCheminPlusCourtPourEviterZoneInondee);
        System.out.println();
        System.out.println("Temps de calcul pour le chemin le plus court évitant la zone inondée : "+d.toMinutes()+"m"+d.toSecondsPart()+"s"+d.toNanosPart()+"ns");
        System.out.println();
        System.out.println("Chemin le plus court pour contourner la zone inondée :");
        System.out.println("------------------------------------------------------");
        int i=1 ;
        for (Localisation l : chemin) {
            System.out.println(i+") "+l.getId()+" | altitude : "+l.getAltitude());
            i++ ;
        }
        System.out.println();
        System.out.println("---------------------------------------------------------------");
        System.out.println();
        start = LocalTime.now() ;
        Map<Localisation,Double> tFlood =  graph.determinerChronologieDeLaCrue(idsDepartCrue,0);
        LocalTime tempsDeterminationChronologieDeLaCrue = LocalTime.now() ;
        d = Duration.between(start,tempsDeterminationChronologieDeLaCrue);
        System.out.println("Temps de calcul pour la chronologie de la crue : "+d.toMinutes()+"m"+d.toSecondsPart()+"s"+d.toNanosPart()+"ns");
        System.out.println();
        System.out.println("Chronologie de la crue : ");
        System.out.println("-------------------------");
        i=1 ;
        for (Localisation l : tFlood.keySet()) {
            System.out.println(i+") "+l.getId()+" | temps inondation : "+ tFlood.get(l));
            i++ ;
        }
        System.out.println();
        System.out.println("---------------------------------------------------------------");
        start = LocalTime.now() ;
        chemin = graph.trouverCheminDEvacuationLePlusCourt(idDepartTrajet,idDestination,10,tFlood) ;
        LocalTime tempsCalculCheminDEvacuationLePlusCourt = LocalTime.now() ;
        d = Duration.between(start,tempsCalculCheminDEvacuationLePlusCourt);
        System.out.println();
        System.out.println("temps de calcul pour le chemin d'évacuation le plus court : "+d.toMinutes()+"m"+d.toSecondsPart()+"s"+d.toNanosPart()+"ns");
        System.out.println();
        System.out.println("Chemin d'évacuation le plus court :");
        System.out.println("-----------------------------------");
        i=1 ;
        for (Localisation l : chemin) {
            System.out.println(i+") "+l.getId()+" | altitude : "+l.getAltitude());
            i++ ;
        }
        System.out.println();
        System.out.println("---------------------------------------------------------------");
    }


}
