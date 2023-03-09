    if (abs(fz) > 1) 
            {
                
                system("clear");
                cout.precision(6);
                cout << "x = " << x(0) <<",\t" <<x(1) << ",\t" <<x(2) << std::endl;
                cout << "f = " << fx << "\t," << fy << "\t," << fz <<endl;
                cout << "m = " << mx << "\t," << my << "\t," << mz <<endl;
                int t_rx, t_ry;
                if(int(x[1]*100) < 0) t_rx = (int)(x[1] *100) + 6;
                else if (int(abs(x[1]*100))== 0) t_rx = 6;
                else if (int(abs(x[1]*100)) > 0)  t_rx = (int)(x[1] *100) + 6;

                if(int(x[0]*100) < 0) t_ry = abs((int)(x[0] *100)) + 6;
                else if (int(abs(x[0]*100))== 0) t_ry =6 ;
                else if (int(abs(x[0]*100)) > 0)  t_ry = 6 - abs((int)(x[0] *100));

              for(int i = 0; i < 11 ; i++)
            {
                for(int j = 0; j < 11; j++)
                {
                    if(abs(t_rx) == (i+1) && abs(t_ry) == (j+1)) 
                    {
                        printf("  ");
                    }
                    else  
                    {
                        printf("██");
                    }
                }
                printf("\n");
            }

                printf("\n");
                printf("\n");
            }
