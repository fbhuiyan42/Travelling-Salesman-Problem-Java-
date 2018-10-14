package pkg1005108;


import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import static java.lang.Math.sqrt;
import java.util.Scanner;
import java.util.Vector;

class Vertex
{
    double x;
    double y;
}

class Graph
{
    Vertex vertex[];
    double weight[][];
    int visited[];
    double distance[];
    int parent[];
    MST frame;
    MST frame2;
    int Holdarray[];
    int count=0;
    
    public Graph(int V)
    {
        vertex = new Vertex[V];
        int i;
        for(i=0;i<V;i++){
            vertex[i]=new Vertex();
        }
        weight = new double[V][V];
        visited = new int[V];
        distance = new double[V];
        parent = new int[V];
        Holdarray= new int[V];
    }

    double eucledean(Vertex a,Vertex b)
    {
        return sqrt( ((a.x - b.x)*(a.x - b.x)) + ((a.y - b.y)*(a.y - b.y)));
    }

    void creategraph(int V)throws IOException
    {
        int i,j,a,b;
        int E=V*(V-1);
        frame=new MST(V);
        frame2=new MST(V);
        for ( i=0;i<V;i++)
        {
            for( j=0;j<V;j++)
            {
                weight[i][j]=0;
            }
        }
        for (i=0;i<V;i++)
        {
            vertex[i].x=Main.in.nextInt();
            vertex[i].y=Main.in.nextInt();
            parent[i]=0;
            visited[i]=0;
            distance[i]=9999;
            int x=(int) vertex[i].x;
            int y=(int) vertex[i].y;
            frame.DrawNode(i,x*10,y*10);
            frame2.DrawNode(i,x*10,y*10);
        }
        for ( a=0;a<V;a++)
        {
            for(b=0;b<V;b++)
            {
               weight[a][b]=weight[b][a]=eucledean(vertex[a],vertex[b]);
            }
        }
    }

    void primMST (int V) throws IOException
    {
      creategraph(V);
      int current,total,i;
      double mincost;
      current=0;
      distance[current]=0;
      total=1;
      visited[current]=1;
      while(total!=V)
      {
        for (i=0;i<V;i++)
        {
          if(visited[i]==0)
              if(weight[current][i]<distance[i])
              {
                  distance[i]=weight[current][i];
                  parent[i]=current;
              }
        }
        mincost=9999;
        for (i=0;i<V;i++)
        {
            if(visited[i]==0)
                if(distance[i]<mincost)
                {
                    mincost=distance[i];
                    current=i;
                }
        }
        visited[current]=1;
        total++;
      }
      mincost=0;
    for(i=0;i<V;i++)
        mincost=mincost+distance[i];
    System.out.print("\n Minimum Spanning tree : ");
    for(i=1;i<V;i++)
    {
        System.out.print("\n " + i + "---" +parent[i]+ "   weight: "+distance[i]);
        frame.from[i]=i;
        frame.to[i]=parent[i];
    }
//System.out.print("\n Minimum cost="+mincost);
    System.out.println();
  }

  int notVisited(int j,int visited[])
  {
       for(int i=0;i<j;i++)
    {
        if(j==visited[i]) return 0;
    }
    return 1;
  }

  void TSP (int V,int i,int visited[],int index)
  {
      System.out.print(" "+i+" ");
      Holdarray[count]=i;
      count++;
      
      for(int k=index;k<V-1;k++) if(visited[k]!=999) index++;
      visited[index]=i;
      for(int j=1;j<V;j++)
        if(parent[j]==i && notVisited(j,visited)==1)
            TSP (V,j,visited,index+1);
  }
  double TSPCost (int V,int visited[])
  {
      double cost=0.0;
      for(int i=0;i<V-1;i++)
      {
          cost=cost + weight[visited[i]][visited[i+1]];
      }
      cost=cost +  weight[visited[V-1]][visited[0]];
      System.out.println("Cost of TSP : "+ cost);
      Main.pw4.print(cost);
      Main.pw4.print(",");
      Main.pw5.print(cost);
      Main.pw5.print(",");
      for(int i=0;i<=V+1;i++){
        if(i==1){
            frame2.from[i-1]=0;
            frame2.to[i-1]=Holdarray[i];
        }
        else if(i>1){
            frame2.from[i-1]=Holdarray[i-2];
            if(i<V+1)frame2.to[i-1]=Holdarray[i-1];
            else frame2.to[i-1]=0;
        }
      }
      
      return cost;
  }
    
  void brute_force (int V) throws IOException 
  {
       Vector<Integer> route = new Vector<>(V);
       Vector<Integer> bestroute = new Vector<>(V);
       for(int i = 0; i < V; ++i)
       {
            route.add(i);
       }
       double[] best = new double[1];
       double[] cost = new double[1];
       best[0]=9999;
       cost[0]=0.0;
       int count[]=new int[1];
       count[0]=0;
       double bestcost=permute(route, new Vector(),best,cost,bestroute,V );
       System.out.print(" TSP : ");
       for(int i = bestroute.size()-V; i<bestroute.size(); ++i)
       {
                   System.out.print(bestroute.get(i)+"  ");
       }
       System.out.print(route.get(0)+"  ");
       System.out.println();
       System.out.println(" Cost : "+ bestcost); 
       Main.pw4.print(bestcost);
       Main.pw4.print(",");
       Main.pw4.println();
  }
  
 double permute(Vector unvisited, Vector visited,double best[],double cost[],Vector<Integer> bestroute,int V)  
 {
     cost[0]=0; 
     if ( unvisited.isEmpty() ) 
      {
	//System.out.println("Permutation: "+visited);
        Integer[] s = (Integer[]) visited.toArray(new Integer[0]);
        for (int i = 0; i < visited.size()-1; i++) 
        {
            cost[0]=cost[0] + weight[s[i+1]][s[i]];
            if (cost[0] > best[0])
                break;
        }
        cost[0]=cost[0] + weight[s[s.length-1]][s[0]];
        if (cost[0] < best[0]) 
        {
            best[0] = cost[0];
            bestroute.addAll(visited);
        }
      }
      else
      { 
	    int l = unvisited.size();
	    for(int i = 0; i<l; i++) 
            {
		int next = (int) unvisited.remove(i);
                visited.add(next);
		permute(unvisited,visited,best,cost,bestroute,V);
		unvisited.add(i,next);;
		visited.removeElement(next);
	    }
      }
      return best[0];
  }
  
 double mst(Vector unvisited)
 {
     double cost=0.0;
     int s=unvisited.size();
     int visited1[]=new int[s];
     double distance1[]=new double[s];
     int parent1[]=new int[s];
     for (int i=0;i<s;i++)
     {
        parent1[i]=0;
        visited1[i]=0;
        distance1[i]=9999;
     }
    int current,total,i;
    double mincost;
    current=0;
    distance1[current]=0;
    total=1;
    visited1[current]=1;
    while(total!=s)
    {
        for (i=0;i<s;i++)
        {
          if(visited1[i]==0)
              if(weight[(int)unvisited.elementAt(current)][(int)unvisited.elementAt(i)]<distance1[i])
              {
                  distance1[i]=weight[current][(int)unvisited.elementAt(i)];
                  parent1[i]=(int)unvisited.elementAt(current);
              }
        }
        mincost=9999;
        for (i=0;i<s;i++)
        {
            if(visited1[i]==0)
                if(distance1[i]<mincost)
                {
                    mincost=distance1[i];
                    current=i;
                }
        }
        visited1[current]=1;
        total++;
      }
    for(i=0;i<s;i++)
        cost = cost+distance1[i];
    //System.out.print("\n MST : ");
    //for(i=1;i<s;i++)
    //    System.out.print("\n " +(int)unvisited.elementAt(i)  + "---" +parent1[i]+ " weight: "+distance1[i]);
    //System.out.print("\n Minimum cost="+ cost);
    //System.out.println();
    return cost;
 }
 
 double getLowerBound(Vector visited,Vector unvisited)
 {
     int a,b,ab;
     double cost=0.0,mina=9999,minb=9999;
     if(visited.size()>1)
     {
        a=(int) visited.elementAt(visited.size()-1);
        b=(int) visited.elementAt(visited.size()-2);
        cost= cost+weight[a][b];
        for(int i=0;i<unvisited.size();i++)
        {
            if(weight[a][(int)unvisited.get(i)] < mina)
            {
                mina= weight[a][i];
            }
            if(weight[b][i] < minb)
            {
                minb= weight[b][i];
            }
        }  
        cost= cost + mina + minb + mst(unvisited);
     }
     return cost;
 }
 
  double pruned(Vector unvisited, Vector visited,double best[],double cost[],Vector bestroute)  
 {
     cost[0]=0; 
     Integer[] s = (Integer[]) visited.toArray(new Integer[0]);
     for (int i = 0; i < visited.size()-1; i++) 
     {
        cost[0]=cost[0] + weight[s[i+1]][s[i]];
     }
     if(s.length>1) cost[0]=cost[0] + weight[s[s.length-1]][s[0]];
     if ( unvisited.isEmpty() ) 
     {
        if (cost[0] < best[0]) 
        {
            best[0] = cost[0];
            bestroute.addAll(visited);
        }
     }
     else if(getLowerBound(visited,unvisited) > best[0]) 
     {
        //System.out.println("pruned");
        return best[0];		
     }
     else
     { 
	 int l = unvisited.size();
	 for(int i = 0; i<l; i++) 
         {
            int next = (int) unvisited.remove(i);
            visited.add(next);
	    pruned(unvisited,visited,best,cost,bestroute);
	    unvisited.add(i,next);;
	    visited.removeElement(next);
	 }
      }
      return best[0];
  }

  void bnb(int V)
  {
       Vector<Integer> route = new Vector<>(V);
       Vector<Integer> bestroute = new Vector<>(V);
       for(int i = 0; i < V; ++i)
       {
            route.add(i);
       }
       double[] best = new double[1];
       double[] cost = new double[1];
       best[0]=9999;
       cost[0]=0.0;
       double bestcost=pruned(route, new Vector(),best,cost,bestroute);
       System.out.print(" TSP : ");
       for(int i = bestroute.size()-V; i<bestroute.size(); ++i)
       {
            System.out.print(bestroute.get(i)+"  ");
       }
       System.out.print(route.get(0)+"  ");
       System.out.println();
       System.out.println(" Cost : "+ bestcost);  
       Main.pw5.print(bestcost);
       Main.pw5.print(",");
       Main.pw5.println();
  }
 
}

public class Main {
    
    public static Scanner in;
    public static PrintWriter pw4;
    public static PrintWriter pw5;

    public static void main(String[] args) throws IOException {
        Graph g;
        //in = new Scanner(new File("ab.txt"));
        in = new Scanner(new File("1005108.txt"));
        PrintWriter pw1 = new PrintWriter(new FileWriter("1005108_1.csv"));
        PrintWriter pw2 = new PrintWriter(new FileWriter("1005108_2.csv"));
        PrintWriter pw3 = new PrintWriter(new FileWriter("1005108_3.csv"));
        pw4 = new PrintWriter(new FileWriter("1005108_4.csv"));
        pw5 = new PrintWriter(new FileWriter("1005108_5.csv"));
        pw1.print("Input Size");
        pw1.print(",");
        pw1.print("Execution Time");
        pw1.print(",");
        pw1.println();
        pw2.print("Input Size");
        pw2.print(",");
        pw2.print("Execution Time");
        pw2.print(",");
        pw2.println();
        pw3.print("Input Size");
        pw3.print(",");
        pw3.print("Execution Time");
        pw3.print(",");
        pw3.println();
        pw4.print("Approximation");
        pw4.print(",");
        pw4.print("Brute_Force");
        pw4.print(",");
        pw4.println();
        pw5.print("Approximation");
        pw5.print(",");
        pw5.print("Branch_Bound");
        pw5.print(",");
        pw5.println();
        while(in.hasNextLine())
        {
            int V=in.nextInt();
            System.out.println("Input Size : "+V);
            pw1.print(V);
            pw1.print(",");
            pw2.print(V);
            pw2.print(",");
            pw3.print(V);
            pw3.print(",");
            System.out.println();
            g=new Graph(V);
            long startTime = System.nanoTime();
            System.out.print("----------Approximation Algorithm--------");
            g.primMST(V);
            int[] visited=new int[V];
            for(int i=0;i<V;i++) visited[i]=999;
            System.out.print(" TSP :");
            System.out.println();
            g.TSP(V,0,visited,0);
            System.out.println(" 0 ");
            g.TSPCost(V,visited);
            long endTime = System.nanoTime();
            long duration = ((endTime - startTime)/1000000);
            System.out.println("execution time : " + duration );
            pw1.print(duration);
            pw1.print(",");
            pw1.println();
            System.out.println();
            System.out.println();
            startTime = System.nanoTime();
            System.out.print("---------Brute Force Algorithm-----------");
            System.out.println();
            g.brute_force(V);
            endTime = System.nanoTime();
            duration = ((endTime - startTime)/1000000);
            System.out.println(" execution time : " + duration );
            pw2.print(duration);
            pw2.print(",");
            pw2.println();
            System.out.println();
            System.out.println();
            startTime = System.nanoTime();
            System.out.print("--------branch & Bound Algorithm----------");
            System.out.println();
            g.bnb(V);
            endTime = System.nanoTime();
            duration = ((endTime - startTime)/1000000);
            System.out.println("execution time : " + duration );
            pw3.print(duration);
            pw3.print(",");
            pw3.println();
            System.out.println();
            System.out.println();
            System.out.println();
        }
          pw1.flush();
          pw1.close();
          pw2.flush();
          pw2.close();
          pw3.flush();
          pw3.close();
          pw4.flush();
          pw4.close();
          pw5.flush();
          pw5.close();
    }
}
