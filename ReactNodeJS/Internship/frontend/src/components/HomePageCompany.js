import React, { Component } from "react";
import { withRouter } from "react-router-dom";
import { Button } from 'react-md';
import ExpDiagram from "./../assets/img/HomePageCompExpDiagram.png";
import UnionSVG from "./../assets/img/Union.svg";
import Arrow from "./../assets/img/Arrow.svg";
import UnionResult from "./../assets/img/UnionResult.svg";
import BundesSVG from "./../assets/img/Bundesminister_Logo.svg";
import IBMSVG from "./../assets/img/IBM_Logo.svg";
import MicrosoftSVG from "./../assets/img/Microsoft_Logo.svg";
import Header from "./Header";
import {Footer} from "./Footer";

class HomePageCompany extends Component {
    constructor(props) {
        super(props);
    }

    render() {
        return (
            <div className="homePageCompMainDiv">
                <Header/>
                <div style={{height:450+'px'}}/>
                <div className="homePageCompDataDiv">
                    <div className="col-md-7">
                        <div className="col-md-2"/>
                        <div className="col-md-10" style={{padding: 20+'px'}}>
                            <div className="col-md-4">
                                <img src={UnionSVG} className="homePageUnionLogo"/>
                                <h4 className="homepageUnionText">Access to a broader network of experienced professionals.</h4>
                            </div>
                            <div className="col-md-2">
                                <img src={Arrow} style={{paddingTop: 20+'px'}}/>
                            </div>
                            <div className="col-md-4">
                                <img src={UnionResult} className="homePageUnionLogo"/>
                                <h4 className="homepageUnionText">Ability of specific in-depth industry and expert knowledge.</h4>
                            </div>
                        </div>
                    </div>
                    <div className="col-md-1"/>
                    <div className="col-md-3">
                        <Button flat primary swapTheming className="positiveBtn homepageCompBtn" onClick={() => this.props.history.push('/company/login')}>Enter your Project</Button>
                        <Button flat primary swapTheming className="positiveBtn homepageCompOtherBtn" onClick={() => this.props.history.push('/company/search')}>See List of Professionals</Button>
                    </div>
                    <div className="col-md-2"/>
                </div>
                <div className="homePageCompExpDiv">
                    <h1 className="text-center">How it Works</h1>
                    <img src={ExpDiagram} className="homePageCompExpDiagram"/>
                    <div style={{marginTop: 3+'em'}}>
                        <div className="col-md-3"/>
                        <div className="col-md-6">
                            <img src={BundesSVG} className="col-md-4 homePageLogo"/>
                            <img src={IBMSVG} className="col-md-4 homePageLogo"/>
                            <img src={MicrosoftSVG} className="col-md-4 homePageLogo"/>
                        </div>
                        <div className="col-md-3"/>
                    </div>
                </div>
                <Footer isSticky={false}/>
            </div>
        );
    }
}

export default withRouter(HomePageCompany);
