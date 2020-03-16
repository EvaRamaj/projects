"use strict";

import React from 'react';
import { Card, Button } from 'react-md';
import { withRouter, Link } from 'react-router-dom';
import {Image} from 'react-bootstrap';
import imgm from "./../assets/img/male_new.png";
import imgf from "./../assets/img/female_new.png";

import 'react-md/dist/react-md.indigo-pink.min.css'

class CompanySearchResults extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
        };
    }

    render() {
        const isCompanyLoggedIn = (this.props.user && this.props.user.role === "company");
        const ResultsGrid = this.props.results.map((rp) =>
            <div className="col-md-4" style={{height: 600+'px'}}>
                <Card>
                    <div style={{textAlign: "center",height:250+'px',paddingTop: 10+'px'}}>
                        {!rp._source.profileData.photo || rp._source.profileData.photo.length===0 || !isCompanyLoggedIn ? [
                            <Image src={imgm} responsive style={{display: "inline-block"}}/>
                        ]
                        :
                        [
                            !rp._source.linkedInData ?
                                [
                                    <Image classname="avatar"
                                           src={require(`../../../backend/uploads/${rp._source.profileData.photo}`)} circle
                                           style={{
                                               display: "inline-block",
                                               border: "1px solid black",
                                               width: 201,
                                               height: 201
                                           }}/>
                                ]
                                :
                                [
                                    <Image classname="avatar"
                                           src={rp._source.profileData.photo} circle
                                           style={{
                                               display: "inline-block",
                                               border: "1px solid black",
                                               width: 201,
                                               height: 201
                                           }}/>
                                ]
                        ]}
                    </div>
                    <div className="text-center" style={{height:300+'px'}}>
                        <h2>{rp._source.profileData.firstName}</h2>
                        <h2>{rp._source.profileData.lastName}</h2>
                        <div style={{height:10+'px'}}/>
                        <h2>{rp._source.profileData.experience[0].title.length>40 ? rp._source.profileData.experience[0].title.substring(0,40)+'...' : rp._source.profileData.experience[0].title}</h2>
                        <h2>{rp._source.profileData.experience[0].compName}</h2>
                        <h3>Industry: {rp._source.profileData.interest.industries}</h3>
                        <h4>Projects: {rp._source.profileData.interest.projects.substring(0,50)+'...'}</h4>
                        <div className="text-center">
                            <Button raised disabled={!isCompanyLoggedIn} primary={isCompanyLoggedIn} iconBefore={false} iconChildren="info" onClick={() => this.props.history.push(`/company/user/${rp._id}`)} tooltipLabel={isCompanyLoggedIn ? "Get Detailed Info" : "Please Register as a Company"}>More Info</Button>
                        </div>
                    </div>
                </Card>
            </div>
        );
        return (
            <div>
                {ResultsGrid}
            </div>
        );
    }
};

export default withRouter(CompanySearchResults);