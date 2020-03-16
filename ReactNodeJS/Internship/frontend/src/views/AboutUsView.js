"use strict";

import React from 'react';
import Page from '../components/Page';
import Card from "react-md/lib/Cards/Card";
import eva from "../assets/img/eva.png";
import yasin from "../assets/img/yasin.png";
import farrukh from "../assets/img/Farrukh .png";
import minsung from "../assets/img/minsung.png";
import rike from "../assets/img/rike.png";
import sara from "../assets/img/sara.png";
import unternemetum from "../assets/img/logo-unternehmertum.png";
import dps from "../assets/img/dps-logo-black.png";
import dehub from "../assets/img/dehub_png_nobg.png";
import bavaria from "../assets/img/Bundesminister_Logo.svg";
import linkedIN from "../assets/img/LinkedIn.png";
import {Image} from "react-bootstrap";



export class AboutUsView extends React.Component {

    constructor(props) {
        super(props);
    }

    render() {
        return (
            <Page isStickyFooter={true}>
                <Card>
                    <div className="col-md-12">
                        <div className="col-md-1"></div>
                        <div className="col-md-10" style={{paddingTop:"2em"}}>
                            <h2>Xperienced.net</h2>
                            <p>We connect you to interesting projects. Because experience never gets old.</p>
                            <h2>WHAT WE DO</h2>
                            <ul>
                                <li>
                                    Connecting - We build an intelligent platform that uses advanced artificial intelligence to match retired professionals to companies.
                                </li>
                                <li>
                                    Empowering - We provide retired professionals with suitable opportunities based on their experience, skills and interests according to companies’ needs.
                                </li>
                                <li>
                                    Changing - Together with our network we grow to change current industry practices and strengthen the collaboration between younger and elder generations.
                                </li>
                            </ul>
                            <h2>MEET OUR TEAM</h2>
                            <p>We are developing xperienced.net together with an international and interdisciplinary team of software developers, AI engineers, interaction designers and product managers.</p>
                        </div>
                        <div className="col-md-1"></div>
                            <div className="col-md-12 text-center" style={{paddingTop:"3em"}}>
                            <div className="col-md-2">
                                <Image classname = "avatar" src={rike} circle style={{display: "inline-block"}}/>
                            </div>
                            <div className="col-md-2">
                                <Image classname = "avatar" src={minsung} circle style={{display: "inline-block"}}/>
                            </div>
                            <div className="col-md-2">
                                <Image classname = "avatar" src={sara} circle style={{display: "inline-block"}}/>
                            </div>
                            <div className="col-md-2">
                                <Image classname = "avatar" src={eva} circle style={{display: "inline-block"}}/>
                            </div>
                            <div className="col-md-2">
                                <Image classname = "avatar" src={farrukh} circle style={{display: "inline-block"}}/>
                            </div>
                            <div className="col-md-2">
                                <Image classname = "avatar" src={yasin} circle style={{display: "inline-block"}}/>
                            </div>

                            <div className="col-md-12 text-center">
                                <div className="col-md-2"><h2>Friederike Möller</h2></div>
                                <div className="col-md-2"><h2>Minsung Cho</h2></div>
                                <div className="col-md-2"><h2>Sara Bajat</h2></div>
                                <div className="col-md-2"><h2>Eva Ramaj</h2></div>
                                <div className="col-md-2"><h2>Farrukh Mushtaq </h2></div>
                                <div className="col-md-2"><h2>Yasin Musa Ayami</h2></div>
                            </div>
                            <div className="col-md-12 text-center">
                                <div className="col-md-2"><h3>Product Manager </h3></div>
                                <div className="col-md-2"><h3>Product Manager </h3></div>
                                <div className="col-md-2"><h3>Interaction Designer </h3></div>
                                <div className="col-md-2"><h3>Software Engineer</h3></div>
                                <div className="col-md-2"><h3>Software Engineer</h3></div>
                                <div className="col-md-2"><h3>AI Engineer</h3></div>
                            </div>
                            <div className="col-md-12 text-center">
                                <div className="col-md-2">
                                    <a href="https://www.linkedin.com/in/friederike-moeller/" target="_blank"><Image classname = "img-fluid" src={linkedIN} style={{width:"70px", height:"30px",display: "inline-block"}}/></a>
                                </div>
                                <div className="col-md-2">
                                    <a href="https://www.linkedin.com/in/chominsung/" target="_blank"><Image classname = "img-fluid" src={linkedIN} style={{width:"70px", height:"30px",display: "inline-block"}}/></a>
                                </div>
                                <div className="col-md-2">
                                    <a href="https://www.linkedin.com/in/sarabajat/" target="_blank"><Image classname = "img-fluid" src={linkedIN} style={{width:"70px", height:"30px",display: "inline-block"}}/></a>
                                </div>
                                <div className="col-md-2">
                                    <a href="https://www.linkedin.com/in/evaramaj/" target="_blank"><Image classname = "img-fluid" src={linkedIN} style={{width:"70px", height:"30px",display: "inline-block"}}/></a>
                                </div>
                                <div className="col-md-2">
                                    <a href="https://www.linkedin.com/in/farrukh-mushtaq/" target="_blank"><Image classname = "img-fluid" src={linkedIN} style={{width:"70px", height:"30px",display: "inline-block"}}/></a>
                                </div>
                                <div className="col-md-2">
                                    <a href="https://www.linkedin.com/in/yasin-musa-ayami/" target="_blank"><Image classname = "img-fluid" src={linkedIN} style={{width:"70px", height:"30px",display: "inline-block"}}/></a>
                                </div>
                            </div>
                        </div>
                        <div className="col-md-12 text-center" style={{backgroundColor:"#E0E0E0",marginTop:"3em"}}>
                            <h2 style ={{paddingTop:"1em"}}>OUR PARTNERS</h2>
                        </div>
                        <div className="col-md-12 text-center" style={{backgroundColor:"#E0E0E0",paddingBottom:"1em"}}>
                            <div className="col-md-3">
                                <a href="http://digitalproductschool.io/index" target="_blank"><Image classname = "img-fluid" src={dps} style={{width:"200px", height:"auto",display: "inline-block"}}/></a>
                            </div>
                            <div className="col-md-3">
                                <a href="https://www.de-hub.de/en/" target="_blank"><Image classname = "img-fluid" src={dehub} style={{width:"200px", height:"auto",display: "inline-block"}}/></a>
                            </div>
                            <div className="col-md-3">
                                <a href="https://www.unternehmertum.de" target="_blank"><Image classname = "img-fluid" src={unternemetum} style={{width:"200px", height:"auto",display: "inline-block"}}/></a>
                            </div>
                            <div className="col-md-3">
                                <a href="https://www.stmwi.bayern.de/en/" target="_blank"><Image classname = "img-fluid" src={bavaria} style={{width:"200px", height:"auto",display: "inline-block"}}/></a>
                            </div>
                        </div>
                        <div className="col-md-12" style={{backgroundColor:"#E0E0E0",paddingBottom:"4em"}}>
                            <p>Please contact us at: <span className="blueText">support@xperienced.net </span>for general inquiries!</p>
                        </div>
                    </div>
                </Card>
            </Page>
        );
    }
}