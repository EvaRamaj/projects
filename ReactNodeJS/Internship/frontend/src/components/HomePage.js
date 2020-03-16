import React, { Component } from "react";
import { withRouter, Link } from "react-router-dom";
import Page from './Page';
import { Button, Media } from 'react-md';
import img1 from "./../assets/img/home.jpg";
import img2 from "./../assets/img/home2.png";

class HomePage extends Component {
    constructor(props) {
        super(props);
    }

    render() {
        return (
            <Page>
                <div className="col-md-12" style={{backgroundImage:'url('+img1+')', backgroundSize: 'cover', height: 750 + 'px'}}>
                    <div className="col-md-1"/>
                    <div className="col-md-6" style={{marginTop: 100 + 'px'}}>
                        <h1>We connect retired Professionals to interesting and tailored projects.</h1>
                        <h3>We use Artificial Intelligence to match you to the best opportunity.</h3>
                        <h3>Have a look at some of our projects.</h3>
                        <div style={{height: 50 + 'px'}}/>
                        <Button flat primary swapTheming iconBefore={false} iconChildren="search" onClick={() => this.props.history.push('/project/search')}>Check Our Projects</Button>
                        <div style={{height: 50 + 'px'}}/>
                        <Button flat secondary swapTheming iconBefore={false} iconChildren="work_outline" onClick={() => this.props.history.push('/company/search')}>For Companies</Button>
                        <div style={{height: 200 + 'px'}}/>
                        <Button flat primary swapTheming iconBefore={false} iconChildren="person_outline" onClick={() => this.props.history.push('/login')} tooltipLabel="As a Retired Professional">Sign Up</Button>
                    </div>
                    <div className="col-md-5"/>
                </div>
                <div className="col-md-12" style={{marginTop: 20 + 'px'}}>
                    <h1 className="text-center">What We Do</h1>
                    <Media aspectRatio="16-9">
                        <img src={img2} alt="Image for Explaining Wayer" />
                    </Media>
                </div>
            </Page>
        );
    }
}

export default withRouter(HomePage);
