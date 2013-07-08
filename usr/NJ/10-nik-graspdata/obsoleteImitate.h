/*
 * obsoleteImitate.h
 *
 *  Created on: Dec 6, 2010
 *      Author: nikolay
 */

#ifndef OBSOLETEIMITATE_H_
#define OBSOLETEIMITATE_H_


#endif /* OBSOLETEIMITATE_H_ */


arr GetDynFeatures(const MT::Array<ors::Shape*> & landmarks,const arr & q, const arr & lastF,const arr & lastQ,ors::Graph * G,arr & grad){
	arr gradR;
	arr raw = GetRawFeaturesJ(landmarks,G,gradR);

	//grad = gradR.sub(0,gradR.d0-1,0,3);
	//return raw.sub(0,3);

	arr TD(raw.N + q.N);
	for(uint i =0 ; i < raw.N; i++)
		TD(i) = raw(i);
	for(uint i =0 ; i < q.N; i++)
		TD(i+raw.N) = q(i);

	arr TD2(TD.N*2);//with added speed from last
	TD2 = 0.0;

	for(uint i = 0; i <  TD.N; i++)
		TD2(i) = TD(i);
	for(uint i = 0; i <  raw.N; i++)
		TD2(i + TD.N) = TD(i) - lastF(i);
	for(uint i = 0; i <  q.N; i++)
		TD2(i + TD.N + raw.N) = TD(i+raw.N) - lastQ(i);

	arr gradF(q.N,TD.N*2);gradF = 0;

	for(uint i = 0; i < q.N; i++ ){
		gradF(i,raw.N+i) = 1;//just joint vector gradients on diagonal
		gradF(i,TD.N+raw.N+i) = 1;
	}
	for(uint i = 0; i < q.N; i++ )
		for(uint j = 0; j < raw.N; j++ ){
			gradF(i,j) = gradR(i,j);
			gradF(i,j+TD.N) = gradR(i,j);
		}

	if(meaN.N == 0){//load arrays
		MT::load(meaN, "discriminant3/mean.txt");
		MT::load(prN, "discriminant3/pr.txt");
	}
	TD2 -=meaN;//mean and scale with projection PCA
	TD2 = TD2*prN;

	grad = gradF*prN;
	return TD2;
}


arr ColumnResize(const arr & a, uint d0, uint d1){
	arr b;
	transpose(b,a);
	arr c = b.resizeCopy(d1,d0);
	transpose(b,c);
	return b;
}

arr MLPa(const arr & feat,const arr & w1,const arr & w2, arr & grad){
	arr l0 = feat*w1.sub(0,w1.d0-2,0,w1.d1-1);
	l0 += w1.sub(w1.d0-1,w1.d0-1,0,w1.d1-1);

	arr sigm = 1./(1.0 + exp(-l0));
	arr sc = sigm*w2.sub(0,w2.d0-2,0,w2.d1-1);
	sc = sc + w2.sub(w2.d0-1,w2.d0-1,0,w2.d1-1);//linear comb + bias
	//return sc;

	arr gradS(sigm.d0,sigm.d0);gradS = 0;
	for (uint i = 0; i < gradS.d0; i++)
		gradS(i,i) = sigm(i)*(1.0-sigm(i));

	grad = w1.sub(0,w1.d0-2,0,w1.d1-1)*gradS*w2.sub(0,w2.d0-2,0,w2.d1-1);
	return sc;
}

arr MLPa(const arr & feat,const arr & w1,const arr & w2){
	arr tmp;
	return MLPa(feat,w1,w2,tmp);
}

double MLPMix(const arr & feat,arr & grad){
	if(w0.N == 0){
		MT::load(w0, "w0.txt");arr tmp = w0.resizeCopy(w0.d0);w0 = tmp;
		MT::load(w12, "w12.txt");
	}
	arr temp = w0.sub(0, (feat.d0+1)*(4+1)-1);
	arr w0a = ColumnResize(temp,feat.d0+1,4+1);
	w0a = w0a.sub(0,w0a.d0-1,0,w0a.d1-2);//last line unused, matlab coding
	temp = w0.sub((feat.d0+1)*(4+1),w0.d0-1);
	arr w0b = ColumnResize(temp,4+1,6+1);
	w0b = w0b.sub(0,w0b.d0-1,0,w0b.d1-2);//lat line unused
	arr g0;
	arr l0 = MLPa(feat,w0a,w0b,g0);

	arr policy(6);
	arr g1(feat.d0,6);
	for(uint i= 0; i < policy.d0; i++){//now mlp policies
		arr temp = w12[i];
		arr tmp0 = temp.sub(0,(feat.d0+1)*(8+1) -1);
		//arr twa = tmp0.resizeCopy(feat.d0+1,8+1);
		arr twa = ColumnResize(tmp0,feat.d0+1,8+1);
		twa = twa.sub(0,twa.d0-1,0,twa.d1-2);//last line unused, matlab coding
		arr twb = temp.sub((feat.d0+1)*(8+1),temp.d0-1);
		arr tgrad;
		policy(i) = MLPsc(feat,twa,twb,tgrad);
		for(uint j = 0; j < tgrad.d0; j++)
			g1(j,i)= tgrad(j);
	}

	grad = g1*l0 + g0*policy;
	return scalarProduct(l0,policy);
}

MT::Array<ors::Shape*> GetLandmarksOLD(ors::Graph * g){
	ors::Shape * oM = g->getBodyByName("m9")->shapes(0);
	ors::Shape * s1 = g->getShapeByName("tipNormal1");
	ors::Shape * s2 = g->getShapeByName("tipNormal2");
	ors::Shape * s3 = g->getShapeByName("tipNormal3");
	ors::Shape * s4 = g->getBodyByName("tip1")->shapes(0);
	ors::Shape * s5 = g->getBodyByName("tip2")->shapes(0);
	ors::Shape * s6 = g->getBodyByName("tip3")->shapes(0);
	ors::Shape * o1 = g->getBodyByName("o1")->shapes(0);
	MT::Array<ors::Shape*> landmarks(8);
	landmarks(0) = oM;
	landmarks(1) = s1;landmarks(2) = s2;landmarks(3) = s3;
	landmarks(4) = s4;landmarks(5) = s5;landmarks(6) = s6;
	landmarks(7) = o1;
	return landmarks;
}

arr GetRawJacobianOLD(const MT::Array<ors::Shape*> & landmarks,ors::Graph * G){
	arr ans(landmarks.N*3,G->getJointStateDimension());
	for(uint i = 0; i < landmarks.N; i++){
		arr J;
		G->jacobianPos(J,landmarks(i)->body->index,&landmarks(i)->rel);
		for(uint j = 0; j < 3; j++)
			ans[i*3+j]= J[j];
	}
	arr ansT; transpose(ansT,ans);
	return ansT;
}

arr GetRawFeaturesOLD(const MT::Array<ors::Shape*> & landmarks){
	arr ans(landmarks.N*3);
	for(uint j = 0; j < landmarks.N; j++){
		ors::Transformation f = landmarks(j)->X;
		ans(j*3) =   f.pos(0);
		ans(j*3+1) = f.pos(1);
		ans(j*3+2) = f.pos(2);
	}
	return ans;
}


arr GetFeatures(const MT::Array<ors::Shape*> & landmarks, const arr & q,ors::Graph * G,arr & grad){
	arr raw = GetRawFeaturesOLD(landmarks);
	arr gradQ = GetRawJacobianOLD(landmarks,G);
	arr gradQ2(gradQ.d0,gradQ.d1 + q.N);gradQ2= 0;//remember converntion, d0 is inputs, d1 - outputs
	for(uint i = 0; i < q.N; i++ )
		gradQ2(i,landmarks.N*3+i) = 1;
	for(uint i = 0; i < gradQ.d0; i++ )
		for(uint j = 0; j < gradQ.d1; j++ )
			gradQ2(i,j) = gradQ(i,j);

	//grad = gradQ;
	//	return raw;


	arr TD(landmarks.N*(landmarks.N-1)/2 + q.N);//translated distances
	uint br = 0;
	arr touples;
	for(uint i = 0; i < landmarks.N; i++)//all endeffectors all landmarks
		for(uint j = i+1; j < landmarks.N; j++)//
		{
			double d = norm(raw.sub(i*3,i*3+2)-raw.sub(j*3,j*3+2));
			TD(br++) = d;
			arr t2(2);
			t2(0) = i;t2(1) = j;
			touples.append(t2);
		}
	for(uint i =0 ; i < q.N; i++)
		TD(i+br) = q(i);
	arr TD2 = TD;
	if(meaN.N == 0){//load arrays
		MT::load(meaN, "mean.txt");
		MT::load(prN, "pr.txt");
	}
	TD2 -=meaN;//mean and scale with projection PCA
	TD2 = TD2*prN;

	arr gradFX(raw.N+q.N,TD.d0);gradFX = 0.0;//features  derivative by raw features
	for(uint i = 0; i < br; i++){
		double c = 1/(2*TD(i));
		arr t2 = touples.sub(i*2,i*2+1);
		for(uint j = 0; j < 3; j++){
			gradFX(t2(0)*3+j,i) = c*2* (raw(t2(0)*3+j)   - raw(t2(1)*3+j)  );
			gradFX(t2(1)*3+j,i) = c*2* (raw(t2(1)*3+j)   - raw(t2(0)*3+j)  );
		}
	}
	for(uint i = 0; i < q.N; i++ )
		gradFX(raw.N+ i ,i+br) = 1;//just joint vector gradients

	grad = gradQ2*gradFX*prN;
	//cout << " grad " << endl << grad << endl << endl << "gradQ2 " << endl << gradQ2 << endl << endl << gradFX << endl;
	return TD2;
}

arr GetFeatures(const MT::Array<ors::Shape*> & landmarks, const arr & q,ors::Graph * G){
	arr a;
	return GetFeatures(landmarks,q,G,a);
}

/*arr GetTaskFeatures(const MT::Array<ors::Shape*> & landmarks, const arr & q){
	arr raw = GetRawFeatures(landmarks);

	//translated distances
	arr TD(landmarks.N*(landmarks.N-1)/2 + q.N);
	int br = 0;
	for(uint i = 0; i < landmarks.N; i++)//all endeffectors all landmarks
		for(uint j = i+1; j < landmarks.N; j++)//
		{
			double d = norm(raw.sub(i*3,i*3+2)-raw.sub(j*3,j*3+2));
			TD(br++) = d;
		}
	for(uint i =0 ; i < q.N; i++)
		TD(i+br) = q(i);

	return TD;
	//2nd poewr features

	arr outer;
	outerProduct(outer, TD,TD);
	br = 0;
	arr uptri(TD.N*(TD.N+1)/2 + TD.N);
	for(uint i = 0; i < TD.N; i++)//get upper ttriangular
		for(uint j = i; j < TD.N; j++)
			uptri(br++) = outer(i,j);
	//add also 1st order features
	for(uint i = 0; i < TD.N; i++)
		uptri(i+br)= TD(i);
	return uptri;
	arr ans2(TD.N*2);
	for(uint i = 0; i < TD.N; i++){
		ans2(i) = TD(i);
		ans2(i+TD.N)= TD(i)*TD(i);
	}

	//cout << " raw " << raw.sub(0,2) << " " <<  raw.sub(21,23) << " n " << norm(raw.sub(0,2)-raw.sub(21,23))<< " " << ans2(6) << endl;
	//cout << " n " << ans2(12) << endl;
	return ans2;
}



arr GetFinalFeatures(const arr& f1,const arr& f2,const arr & alfa){
	/*arr concat(f1.d0*2);
	for (uint i = 0; i < f1.d0; i++){
		concat(i)=f1(i);
		concat(i+f1.d0) = f2(i);
	}
	arr concat = f1;
	if(meaN.N == 0){//load arrays
		MT::load(meaN, "mean.txt");
		MT::load(prN, "pr.txt");
	}
	concat -=meaN;//mean and scale with projection PCA
	concat = concat*prN;

	return concat;

	return concat.sub(0,concat.d0/2-1);//off with second feature for now

	//add alfa mixture at the end
	arr concatA(concat.N + alfa.N + 1);
	for (uint i = 0; i < concat.N; i++)
		concatA(i)= concat(i);
	for(uint i = concat.N; i < concat.N + alfa.N; i++)
		concatA(i) = alfa(i-concat.N);
	concatA(concatA.N-1)=1;
	return concatA;
}*/

/*
double EnergyCost(const arr & feat, arr & alfa){
	return MLPMix(feat);
	return feat(12);
	if(w.N == 0){
		MT::load(w, "w.txt");
	}
	uint K = w.d1;
	uint s = (feat.d0 -K-1)/2;
	double total2;
	arr total(s,1);total = 0;
	alfa = arr(K);
	for(uint k = 0; k < K; k++){
		arr wt = w.sub(s,w.d0-1,k,k);
		//arr wtt;transpose(wtt,wt);
		arr ft = feat.sub(s,w.d0-1);
		double sc = scalarProduct(wt,ft);
		//	cout << wt << endl << ft << endl << sc << endl;
		alfa(k) = exp(sc);
		total2 += alfa(k);
		total += alfa(k)*w.sub(0,s-1,k,k);
	}
	total /= total2;
	alfa /= total2;
	double ans = scalarProduct(total,feat.sub(0,s-1));
	return ans;
}*/
/*
%call batch optimizer
if false
    clear vv; clear allW;
    hyper = cell(5,1);
    hyper{1}= T;hyper{2} = 0.01;hyper{3} = 0.0;
    ind = 1:size(z,1);%%all data points
    indTr = find(ind > T*noi*nTr);
    ind(indTr) = [];
    [hyper{5} policies sgreed] = PreparePolicies(z(:,:),eucl(:,:),noi,T,NN,nTr);
    return;
    if false
        feCut = sum(abs(policies));
        [soCut raCut] = sort(-feCut);
        goodCut = raCut(1:420);    %plot
    end
    for co = 1:20
        if true
            hyper{4} = 0.005;
            w = rand( ((size(z,2))+K+1)*K,1)-0.5;
            wnew = reshape(w,[],K);
            for i = 1:K
                wnew(1:(size(z,2))/2,i) = policies(sgreed(i),:)';
            end
            w = reshape(wnew,[],1);            % w = w0+ randn(size(w0))*0.01;
            options = optimset('GradObj','on','Display','iter',...
                'MaxIter',100,'DerivativeCheck','off','LargeScale','off');
            start = ones(1,K)/K;%always start from this prototype in recursion
            f = @(x)MixtureEnergyAlfaFix(z(ind,:),noi,x,hyper);
            h = @(x)MixtureEnergy(z(ind,:),noi,x,eucl(1:nTr*T,:),hyper);
            for i = 1:1
                [w v]= fminunc(f,w,options);
                [w v]= fminunc(h,w,options);
            end
            %             clear rprop; rprop('init',0.1);
            %             for i = 1:100
            %                 [v g] = h(w);
            %                 rprop('step',w,g);
            %                 v
            %             end
        else
            load wgood;
        end
        hyper{4} = 0.0;
        allW(co,:) = w;
        [vv(co) g alfa]=  MixtureEnergy(z(:,:),noi,w,eucl(:,:),hyper);
    end
end


%mlp of mlps
if false
    %police = load('mlp6k8f38.mat') ;
    %police = police.w';
    nK = 5;
    police = policies(sgreed(1:nK),:);

    w =  rand( (size(z,2)+1)*(K+1) + (K+1)*(nK+1),1)-0.5;

    w = [w' reshape(police,[],1)']';


    ind = 1:size(z,1);%%all data points
    indTr = find(ind > T*noi*nTr);
    ind(indTr) = [];

    options = optimset('GradObj','on','Display','iter',...
        'MaxIter',100,'DerivativeCheck','off','LargeScale','off');
    hyper = cell(2,1);
    hyper{1}= T;hyper{2} = 0.0005;hyper{3} = 5;
    h = @(x)MixtureMLP(z(ind,:),noi,x,eucl(1:nTr*T,:),hyper);
    [w v]= fminunc(h,w,options);
    MixtureMLP(z(:,:),noi,w,eucl(:,:),hyper);

    w0 = w(1: (size(z,2)+1)*(K+1) + (K+1)*(nK+1))';
    police = w((size(z,2)+1)*(K+1) + (K+1)*(nK+1)+1:size(w,1));
    police = reshape(police,[],nK)';
    save('w12.txt','police','-ascii');
    save('w0.txt','w0','-ascii');
    save('pr.txt','Pr','-ascii');
    save('mean.txt','mi','-ascii');
    return
end

if false
    [s sx] = sort(vv);
    w = allW(sx(1),:)';
    [val g alfa]=  MixtureEnergy(z(:,:),noi,w,eucl(:,:),hyper);

    wnew = reshape(w,[],K);
    save('w.txt','wnew', '-ascii');
    PRS = diag(ss)*Pr;
    save('pr.txt','PRS','-ascii');
    save('mean.txt','mi','-ascii');
    save('start.txt','start','-ascii');

end*/
