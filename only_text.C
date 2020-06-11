Double_t roundToThousandths(Double_t x){
    
	x*=1000;
	return floor(x+0.5)/1000;
    
}

void only_text(int RunNumber){
    
    // select point
    Int_t MgPoint = 0; // only phidg0
    //Int_t MgPoint = 1; // phidg 0,3,4
    //Int_t MgPoint = 2;
    
    // outputfile
    ofstream outputfile(Form("Magnetic_%d_phidg4.txt",RunNumber));
    
    //Set up File I/O
    TFile* f = new TFile(Form("~/online/rootfiles/out_run0%d.root",RunNumber));
    
    Int_t numEntries = 0;
    Double_t xposition = 0.;
    Double_t yposition = 0.;
    Double_t zposition = 0.;
    
    Double_t xBfield_0[100] = {0.};
    Double_t yBfield_0[100] = {0.};
    Double_t zBfield_0[100] = {0.};
    
    Double_t xBfield_3[100] = {0.};
    Double_t yBfield_3[100] = {0.};
    Double_t zBfield_3[100] = {0.};
    
    Double_t xBfield_4[100] = {0.};
    Double_t yBfield_4[100] = {0.};
    Double_t zBfield_4[100] = {0.};
    
    //Creates pointer to TTree contained in the input file and pointers to its relevant branches
    TTree* T = (TTree*)f->Get("scan_tree");
    TBranch *branch = T->GetBranch("phidg0_By");
    
    TBranch *gantry0_x = T->GetBranch("gantry0_x");
    TBranch *gantry0_y = T->GetBranch("gantry0_y");
    TBranch *gantry0_z = T->GetBranch("gantry0_z");
    TBranch *phidg0_Bx = T->GetBranch("phidg0_Bx");
    TBranch *phidg0_By = T->GetBranch("phidg0_By");
    TBranch *phidg0_Bz = T->GetBranch("phidg0_Bz");
    
    TBranch *phidg3_Bx = T->GetBranch("phidg3_Bx");
    TBranch *phidg3_By = T->GetBranch("phidg3_By");
    TBranch *phidg3_Bz = T->GetBranch("phidg3_Bz");
    
    TBranch *phidg4_Bx = T->GetBranch("phidg4_Bx");
    TBranch *phidg4_By = T->GetBranch("phidg4_By");
    TBranch *phidg4_Bz = T->GetBranch("phidg4_Bz");
    
    //defines total number of entries for indexing bound
    numEntries = branch->GetEntries();
    
    //Sets where the value read in by GetEntry() is stored
    gantry0_x->SetAddress(&xposition);
    gantry0_y->SetAddress(&yposition);
    gantry0_z->SetAddress(&zposition);
    
    phidg0_Bx->SetAddress(&xBfield_0);
    phidg0_By->SetAddress(&yBfield_0);
    phidg0_Bz->SetAddress(&zBfield_0);
    
    phidg3_Bx->SetAddress(&xBfield_3);
    phidg3_By->SetAddress(&yBfield_3);
    phidg3_Bz->SetAddress(&zBfield_3);
    
    phidg4_Bx->SetAddress(&xBfield_4);
    phidg4_By->SetAddress(&yBfield_4);
    phidg4_Bz->SetAddress(&zBfield_4);
    //Specify the number of decimal point
    outputfile<< fixed << setprecision(3);
    
    //looping code to iterate through the TTree Structure
    
    for(Long64_t i = 0; i < numEntries; i++){
        T->GetEntry(i);
        // output of text
        switch (MgPoint) {
            case 0: //phidg0
                outputfile
                << roundToThousandths(xposition)<<", "
                << roundToThousandths(yposition)<<", "
                << roundToThousandths(zposition+0.5)<<", "
                << roundToThousandths(xBfield_4[1])<<", "
                << roundToThousandths(yBfield_4[1])<<", "
                << roundToThousandths(zBfield_4[1])<<"\n";
		//cout << "0" << endl;

                break;
            case 1: // phidg0,3,4
                outputfile
                << roundToThousandths(xposition)<<", "
                << roundToThousandths(yposition)<<", "
                << roundToThousandths(zposition)<<", "
                << roundToThousandths(xBfield_0[1])<<", "
                << roundToThousandths(yBfield_0[1])<<", "
                << roundToThousandths(zBfield_0[1])<<"\n"
                
                << roundToThousandths(xposition + 0.00)<<", "
                << roundToThousandths(yposition + 0.05)<<", "
                << roundToThousandths(zposition + 0.1)<<", "
                << roundToThousandths(xBfield_3[1])<<", "
                << roundToThousandths(yBfield_3[1])<<", "
                << roundToThousandths(zBfield_3[1])<<"\n"
                
                << roundToThousandths(xposition + 0.00)<<", "
                << roundToThousandths(yposition + 0.05)<<", "
                << roundToThousandths(zposition + 0.5)<<", "
                << roundToThousandths(xBfield_4[1])<<", "
                << roundToThousandths(yBfield_4[1])<<", "
                << roundToThousandths(zBfield_4[1])<<"\n";

            	//cout << "0,3,4" <<endl; 
		break ;
		default:

                cout << "error" <<endl;
                break;
        }
        
    }
    outputfile.close();
}



