"use strict";

import React from 'react';
import { withRouter} from 'react-router-dom'
import {Button, Card} from 'react-md';

import Page from './Page';
import {Image} from "react-bootstrap";
import imgm from "../assets/img/male_new.png";

//var Mailto = require('react-mailto');

const style = { maxWidth: 700, paddingLeft: 80, paddingRight: 80, paddingBottom: 20};

export class UserDetails extends React.Component {

    constructor(props) {
        super(props);
    }

    render() {
        let user = this.props.user._source;
        return (
            <Page>
                <Card style={style} className="md-block-centered">
                    <div style={{textAlign: "center", border: 20}}>
                        {!user.profileData.photo ?
                            [
                                <Image classname = "avatar" src={imgm} circle style={{display: "inline-block", border: "1px solid black"}}/>
                            ]
                            :
                            [ !user.linkedInData ?
                                [
                                    <Image classname="avatar"
                                           src={require(`../../../backend/uploads/${user.profileData.photo}`)} circle
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
                                           src={user.profileData.photo} circle
                                           style={{
                                               display: "inline-block",
                                               border: "1px solid black",
                                               width: 201,
                                               height: 201
                                           }}/>
                                ]
                            ]
                        }
                    </div>
                    <div className= " text-center UserInformation" >
                        <p className="text-center" style={{ fontWeight: 'bold' }}> {user.profileData.prefix} {user.profileData.firstName} {user.profileData.lastName}</p>
                        <p className="text-center" style={{ fontWeight: 'bold' }}>{user.profileData.experience[0].title + " in " + user.profileData.experience[0].compName}</p>
                    </div>
                    <div className= "text-left Summary">
                        <h4  style={{ fontWeight: 'bold' }}>SUMMARY</h4>
                        <p>A passionate entrepreneur who gets a rush from helping others succeed, be it a start-up, small business, or Fortune 500 company, Strati excels in serving his clients’ needs to overcome barriers to growth through Executive Search & Placement, and innovative Marketing Strategies that produce results.
                        </p>
                    </div>
                    <div className= "text-left Specialities">
                        <h4  style={{ fontWeight: 'bold' }}>SPECIALITIES</h4>
                        <p>Focused on strategic business development in the areas of Automation and Motion, Robotics, Additives and Digitalization. Internationally astute with credible success in leading global sales, service, product development. Entrepreneurial and decisive - highly adapt at identifying new markets and growing market share. Firm belief in collaboration and talent development. Have mentored several senior leaders over many years. Looking forward to helping companies achieve their aggressive growth targets.</p>
                    </div>
                    <div className= "text-left Interests">
                        <h4  style={{ fontWeight: 'bold' }}>INTERESTS</h4>
                        <p className="text-left"> Industries: {user.profileData.interest.industries}</p>
                        <p className="text-left"> Projects: {user.profileData.interest.projects}</p>
                    </div>
                    <div className="text-center" style={{paddingTop: 20+'px'}}>
                        <Button raised iconBefore={false} iconChildren="message" className="figmaButton" onClick={() => this.props.history.push({pathname:'/company/user/contact',state:{user: this.props.user}})}>Send a Message</Button>
                    </div>
                </Card>
            </Page>
        );
    }
}
export default withRouter(UserDetails);
