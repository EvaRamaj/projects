"use strict";

import React from 'react';
import {Button, Card} from 'react-md';
import { withRouter } from 'react-router-dom';

import 'react-md/dist/react-md.indigo-pink.min.css'
import {Image} from "react-bootstrap";
import imgm from "../assets/img/male_new.png";
import Page from "./Page";
import StarRatings from 'react-star-ratings';

class MatchingResults extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
        };
    }

    render() {
        const ResultsGrid = this.props.results.map((user,index) =>
            <div className="col-md-4" style={{height: 600+'px'}}>
                <Card className="block" style={{height:"50em"}}>
                    <div style={{textAlign: "center",height:250+'px',paddingTop: 10+'px'}}>
                        {!user._source.profileData.photo || user._source.profileData.photo.length===0 ? [
                                <Image src={imgm} responsive style={{display: "inline-block"}}/>
                            ]
                            :
                            [
                                !user._source.linkedInData ?
                                    [
                                        <Image classname="avatar"
                                               src={require(`../../../backend/uploads/${user._source.profileData.photo}`)} circle
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
                                               src={user._source.profileData.photo} circle
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
                        <StarRatings
                            rating={this.props.users[index].normal_score*5}
                            numberOfStars={5}
                            starRatedColor = "rgb(255,255,0)"
                            starDimension="40px"
                            starSpacing="15px"
                        />
                        <h2>{user._source.profileData.firstName}</h2>
                        <h2>{user._source.profileData.lastName}</h2>
                        <div style={{height:10+'px'}}/>
                        <h2>{user._source.profileData.experience[0].title.length>40 ? user._source.profileData.experience[0].title.substring(0,40)+'...' : user._source.profileData.experience[0].title}</h2>
                        <h2>{user._source.profileData.experience[0].compName}</h2>
                        <h3>Industry: {user._source.profileData.interest.industries}</h3>
                        <h4>Projects: {user._source.profileData.interest.projects.substring(0,50)+'...'}</h4>
                        <div class="text-center" style={{marginTop:"5em"}}>
                            <Button raised secondary iconBefore={false} iconChildren="info" onClick={() => this.props.history.push(`/company/user/${user._id}`)} >More Info</Button>
                        </div>
                    </div>
                </Card>
            </div>
        );
        return (
            <Page isStickyFooter={true}>
                <div className="text-center col-md-12" style={{paddingTop:"3em",paddingBottom: 10 + 'px', backgroundColor:"#E6F4F6", height:"100vh"}}>
                    <h2>These are the matches for this project:  </h2>
                    {ResultsGrid}
                </div>
            </Page>
        );
    }
};

export default withRouter(MatchingResults);