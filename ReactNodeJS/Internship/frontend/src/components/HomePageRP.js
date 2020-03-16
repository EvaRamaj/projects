import React, { Component } from "react";
import { withRouter } from "react-router-dom";
import { Button } from 'react-md';
import HomePageImg from "./../assets/img/HomePageRP.jpg";
import BundesSVG from "./../assets/img/Bundesminister_Logo.svg";
import IBMSVG from "./../assets/img/IBM_Logo.svg";
import MicrosoftSVG from "./../assets/img/Microsoft_Logo.svg";
import DiagramSVG from "./../assets/img/homePageRPDiagram.svg";
import Header from "./Header";
import {Footer} from "./Footer";

class HomePageRP extends Component {
    constructor(props) {
        super(props);
    }

    render() {
        return (
            <div className="homePageRPMainDiv">
                {/*<img src={HomePageImg} className="bg"/>*/}
                <Header/>
                <div style={{height:400+'px'}}/>
                <div className="homePageRPDataDiv">
                    <div className="col-md-6">
                        <div className="col-md-1"/>
                        <div className="col-md-11">
                            <h1 style={{paddingTop:60+'px',fontSize: 40+'px'}}>We connect you to interesting projects, because experience never gets old.</h1>
                            <h4 style={{marginTop: 100+'px'}}>Supported by the Minister of State for Digitization.</h4>
                            <h4 style={{marginBottom: 50+'px'}}>In Partnership with IBM and Microsoft.</h4>
                        </div>
                        <img src={BundesSVG} className="col-md-4 homePageLogo"/>
                        <img src={IBMSVG} className="col-md-4 homePageLogo"/>
                        <img src={MicrosoftSVG} className="col-md-4 homePageLogo"/>
                    </div>
                    <div className="col-md-6">
                        <div className="col-md-3"/>
                        <div className="col-md-4">
                            <Button  className = "getInvolved" onClick={() => this.props.history.push('/login')}>GET INVOLVED</Button>
                        </div>
                        <div className="col-md-4">
                            <Button className = "seeProjects" onClick={() => this.props.history.push('/project/search')}>SEE PROJECTS</Button>
                        </div>
                    </div>
                    <div className="col-md-6"/>
                    <div className="col-md-6" style={{marginTop:"5em"}}>
                        <div className="col-md-3"/>
                        <div className="col-md-3">
                            <img src={DiagramSVG}/>
                        </div>
                    </div>
                    <div className="col-md-6"/>
                    <div className="col-md-6" >
                        <div className="col-md-2"/>
                        <div className="col-md-2"/>
                        <div className="col-md-2 text-center" style={{marginTop:"1em"}}>
                            <p>Simply enter your experiences, interests and skills.</p>
                        </div>
                        <div className="col-md-2 text-center" style={{marginTop:"1em", marginLeft:"1.5em"}}>
                            <p>Project owners will approach you to connect with you.</p>
                        </div>
                        <div className="col-md-2"/>
                        <div className="col-md-2"/>
                    </div>
                </div>
                <Footer isSticky={false}/>
            </div>
        );
    }
}

export default withRouter(HomePageRP);
